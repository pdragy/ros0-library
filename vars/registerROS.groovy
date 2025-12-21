def call(def docker_tag, def build_deps, def run_deps) {
    def image
    def ros_version
    def set_env = ". /opt/ros/\$ROS_DISTRO/setup.sh"
    def build_script = libraryResource 'build.sh'
    def git_commit  = sh(script: "git rev-parse --short HEAD", returnStdout: true).trim()
    def n_commits = sh(script: "git rev-list --count HEAD", returnStdout: true).trim()
    def pkg_release = "${n_commits}.${BUILD_NUMBER}.${git_commit}"
    def pkg_requires = "'" + run_deps.join(',') + "'"

    stage ("Docker Pull") {
        image = docker.image("roszero/" + docker_tag)
        image.pull()
        image.inside() {
            ros_version = sh(script: "$set_env && echo \$ROS_VERSION", returnStdout: true).trim()
        }
    }

    def build_cmd = ros_version == "2" ? "colcon build --merge-install" : "catkin_make_isolated --source ."
    def test_cmd =  ros_version == "2" ? "colcon test --merge-install"  : "catkin_make_isolated --source . --catkin-make-args run_tests"

    image.inside("-u root") {
        stage ("Build + Test") {
            sh "apt-get install " + build_deps.join(' ')
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

    stage ("Deploy .deb") {
        //docker.image = docker.image("deb-s3").inside() {
        //sh "deb-s3 upload **/*$pkg_release*.deb -c \$(lsb_release -cs) --bucket foo --visibility=nil --sign=foo --gpg-options \"\\-\\-pinentry-mode=loopback \\-\\-passphrase foo"
        //}
    }
}
