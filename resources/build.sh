#!/bin/sh

# Hack to short circuit the ros package build, first stake to break dependency on rosdep

if [ "$#" -lt 3 ]; then
    echo "Usage: build.sh <release> <comma-separated-requires> <yes|no>"
    exit
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "Make sure you have set up your ROS environment!"
    exit
fi

pkg_release=${1}
reqs=${2}
fstrans=${3}

# no glob
set -f

ros_home=/opt/ros/$ROS_DISTRO
#ros_version=`.$ROS_VERSION`

if [ "$ROS_VERSION" = "2" ] ; then
    build_cmd="colcon build --event-handlers console_direct+ --merge-install --install-base $ros_home --cmake-args -DCMAKE_BUILD_TYPE=Release"
    exclude_files="$ros_home/.colcon_install_layout,$ros_home/COLCON_IGNORE,$ros_home/_local_setup_util_*.py,$ros_home/local/lib/python*/dist-packages/rosidl*,$ros_home/local_setup.*,$ros_home/setup.*,**/*.pyc"
else
    rm -rf devel_isolated build_isolated
    build_cmd="catkin_make_isolated --source . --install --install-space $ros_home -DCMAKE_BUILD_TYPE=Release"
    exclude_files="$ros_home/.rosinstall,$ros_home/_setup_util.py,$ros_home/env.sh,$ros_home/local_setup.*,$ros_home/setup.*,**/*.pyc"
fi

#maintainer_email=`grep -Po '(?<=maintainer email=)[^>]+' package.xml`
pkg_name=`grep -Po '(?<=name>)[^<]+' package.xml | tr '_' '-'`
pkg_version=`find . -name package.xml | xargs -L1 cat | grep -Po '(?<=version>)[^<]+' | sort -rn | head -1`
if [ -z "$pkg_version" ]; then
    pkg_version='0.0.0'
fi

if [ -z "$pkg_name" ]; then
   if [ "$ROS_VERSION" = "2" ] ; then
        pkg_name="unknown"
    else
        pkg_name=`catkin_topological_order | head -1 | cut -f 1 -d ' '`
   fi
fi

. /opt/ros/$ROS_DISTRO/setup.sh && checkinstall -y \
    --backup=no \
    --deldesc=yes \
    --exclude=$exclude_files \
    --pkgname=ros-$ROS_DISTRO-$pkg_name \
    --pkgversion=$pkg_version \
    --pkgrelease=$pkg_release \
    --requires=${reqs} \
    --pkggroup=none \
    --install=no \
    --nodoc \
    --fstrans=$fstrans \
    $build_cmd
