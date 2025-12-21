def call(def docker_image, def build_deps, def run_deps) {
    def ros_distro
    def image
    def short_commit
    //def pkg_name
    //def version = sh(script: 'cat package.xml | grep -Po \'(?<version>)[0-9]*\\.[0-9]*\\.[0-9]*\'', returnStdout: true).trim()
    //def pkg_name_split = currentBuild.fullProjectName.split('/')[-2]
    //pkg_name = pkg_name_split.replaceAll('_','-').toLowerCase()
    //pkg_release = 

    short_commit = scm_vars.GIT_COMMIT[0..6]

    stage ("Docker Pull") {
        image = docker.image(docker_image)
        image.pull()
        image.inside() {
            //arch = sh(script: 'gcc -dumpmachine', returnStdout: true).trim()
            ros_distro = sh(script: 'echo $ROS_DISTRO', returnStdout: true).trim()
        }
    }

    stage ("Build and Test") {
        image.inside() {
            sh "sudo apt update && sudo apt install -y " + build_deps.join(' ')
            try {
                sh ". /opt/ros/$ros_distro/setup.sh && colcon build --merge-install --event-handlers console_direct+"
                try {
                    sh ". /opt/ros/$ros_distro/setup.sh && colcon test --merge-install --event-handlers console_direct+"
                } catch (err) {
                    currentBuild.result = 'UNSTABLE'
                }
                junit skipPublishingChecks: true, testResults: "build/*/test_results/**/*.xml", allowEmptyResults: true
            } catch (err) {
                sh "sudo rm -rf build install log devel logs"
                currentBuild.result = 'FAILED'
                sh "exit 1"
            }
            sh "sudo rm -rf build install log devel logs"
        }
    }

    //stage ("Build .deb") {
    //    pkg_release = sh(script: "git rev-list --count HEAD", returnStdout: true).trim() + '-' branch_name + '-' + short_commit// + "-$BUILD_NUMBER"
    //    try {
    //        buildDeb(ecr, image, pkg_name, pkg_release, pkg_requires)
    //    } catch (err) {
    //        image.inside() {
    //            sh "sudo rm -rf build install log"
    //        }
    //        currentBuild.result = 'FAILED'
    //        sh "exit 1"
    //    }
    //    image.inside() {
    //        sh "sudo rm -rf build install log"
    //    }
    //}

    //stage ("Deploy .deb") {
    //    deployDeb(pkg_name, branch_name, ros_distro, arch)
    //}
}
