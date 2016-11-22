#!/bin/bash

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -o errexit
set -o verbose

sudo apt-get update

source /ros_entrypoint.sh

# Create a new workspace in 'catkin_ws'.
mkdir catkin_ws
cd catkin_ws
wstool init src

mv ../cartographer_turtlebot src

# Install rosdep dependencies.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build, install, and test.
#
# It's necessary to use the '--install' flag for every call to
# 'catkin_make_isolated' in order to avoid the use of 'devel_isolated' as the
# 'CMAKE_INSTALL_PREFIX' for non-test targets. This in itself is important to
# avoid any issues caused by using 'CMAKE_INSTALL_PREFIX' during the
# configuration phase of the build.
export BUILD_FLAGS="--use-ninja
                    --install-space /opt/cartographer_turtlebot
                    --install"
catkin_make_isolated ${BUILD_FLAGS}
catkin_make_isolated ${BUILD_FLAGS} --catkin-make-args run_tests

# Clean up.
cd ..
rm -rf catkin_ws /var/lib/apt/lists/*
