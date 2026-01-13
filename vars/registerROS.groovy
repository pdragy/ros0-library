def call(def docker_tag, def build_deps, def run_deps) {
    def image
    def ros_version
    def set_env = ". /opt/ros/\$ROS_DISTRO/setup.sh"
    def build_script = libraryResource 'build.sh'
    def git_commit  = sh(script: "git rev-parse --short HEAD", returnStdout: true).trim()
    def n_commits = sh(script: "git rev-list --count HEAD", returnStdout: true).trim()
    def pkg_release = "${n_commits}.${BUILD_NUMBER}.${git_commit}"
    def pkg_requires = "'" + run_deps.join(',') + "'"
    def pkg_prefix
    def pkg_component

    stage ("Docker Pull") {
        image = docker.image("roszero/" + docker_tag)
        image.pull()
        image.inside() {
            ros_version = sh(script: "$set_env && echo \$ROS_VERSION", returnStdout: true).trim()
            pkg_prefix = sh(script: "lsb_release -is | tr A-Z a-z", returnStdout: true).trim()
            pkg_component = sh(script: "lsb_release -cs", returnStdout: true).trim()
        }
    }

    def build_cmd = ros_version == "2" ? "colcon build --event-handlers console_direct+ --merge-install" : "catkin_make_isolated --source ."
    def test_cmd =  ros_version == "2" ? "colcon test --event-handlers console_direct+ --merge-install"  : "catkin_make_isolated --source . --catkin-make-args run_tests"
      set(ament_cmake_cpplint_FOUND TRUE)


    image.inside("-u root") {
        stage ("Build + Test") {
            if (build_deps) {
                sh "apt-get update && apt-get install -y " + build_deps.join(' ')
            }
            sh "$set_env && $build_cmd"
            try { sh "$set_env && $test_cmd" } catch (err) { currentBuild.result = 'UNSTABLE' }
            junit skipPublishingChecks: true, testResults: "**/test_results/**/*.xml", allowEmptyResults: true
        }
        stage ("Build .deb") {
            writeFile file: 'build.sh', text: build_script
            sh "chmod +x ./build.sh && $set_env && ./build.sh $pkg_release $pkg_requires no"
            sh "sh -c 'find . -name *$pkg_release*.deb | xargs dpkg-deb -c ' | tee ~package_list.txt"
            archiveArtifacts artifacts: "**/*$pkg_release*.deb,~package_list.txt", fingerprint: true, allowEmptyArchive: false
        }
    }

    try {
        withCredentials([string(credentialsId: 'gpg-id')]) {}
    } catch (err) {
        echo "Skipping deploy stage"
        return
    }

    stage ("Deploy .deb") {
        withCredentials([string(credentialsId: 'gpg-id', variable: 'GPGID')]) {
            withCredentials([string(credentialsId: 'gpg-pass', variable: 'GPGPASS')]) {
                docker.image("roszero/deb-s3:latest").inside("-v /mnt/efs/gpg/.gnupg:/root/.gnupg -u root --entrypoint=") {
                    try {
                        sh "deb-s3 upload **/*${pkg_release}*.deb --prefix=${pkg_prefix} -c ${pkg_component} --bucket ros0-repo --s3-region=us-west-2 --lock -e --visibility=bucket_owner --sign=" + '${GPGID} --gpg-options "\\-\\-pinentry-mode=loopback \\-\\-passphrase ${GPGPASS}"'
                    } catch (err) {
                        echo "Failed to deploy .deb package!"
                        currentBuild.result = 'FAILED'
                    }
                    sh "deb-s3 clean --bucket ros0-repo --s3-region us-west-2 --prefix $pkg_prefix -c $pkg_component"
                }
            }
        }
    }
}
