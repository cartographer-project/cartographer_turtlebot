.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

===============================
Cartographer ROS for TurtleBots
===============================

.. toctree::
   :maxdepth: 2
   :hidden:

`Cartographer`_ is a system that provides real-time simultaneous localization
and mapping (`SLAM`_) in 2D and 3D across multiple platforms and sensor
configurations. This repository provides Cartographer SLAM for `TurtleBots`_
via `Cartographer ROS`_.

.. _Cartographer: https://github.com/googlecartographer/cartographer
.. _Cartographer ROS: https://github.com/googlecartographer/cartographer_ros
.. _SLAM: https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping
.. _TurtleBots: http://www.turtlebot.com/

Building & Installation
=======================

Installation has been tested on Ubuntu 14.04 (Trusty) with ROS Indigo, but may
also work on Ubuntu 16.04 (Xenial) with ROS Kinetic. We recommend using
`wstool <http://wiki.ros.org/wstool>`_ and
`rosdep <http://wiki.ros.org/rosdep>`_. For faster builds, we also recommend
using `Ninja <https://ninja-build.org>`_.

  .. code-block:: bash

    # Install wstool and rosdep.
    sudo apt-get update
    sudo apt-get install -y python-wstool python-rosdep ninja-build

    # Create a new workspace in 'catkin_ws'.
    mkdir catkin_ws
    cd catkin_ws
    wstool init src

    # Merge the cartographer_turtlebot.rosinstall file and fetch code for dependencies.
    wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_turtlebot/master/cartographer_turtlebot.rosinstall
    wstool update -t src

    # Install deb dependencies.
    rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

    # Build and install.
    catkin_make_isolated --install --use-ninja
    source install_isolated/setup.bash

Running the demo
================

Now that Cartographer, Cartographer ROS, and Cartographer ROS's TurtleBot
integration are installed, download the example bag to a known location, in
this case ``~/Downloads``, and use ``roslaunch`` to bring up the demo:

  .. code-block:: bash

    # Download the example bag.
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/turtlebot/cartographer_turtlebot_demo.bag

    # Launch the 2D LIDAR demo.
    roslaunch cartographer_turtlebot demo_lidar_2d.launch bag_filename:=${HOME}/Downloads/cartographer_turtlebot_demo.bag

    # Launch the 2D depth camera demo.
    roslaunch cartographer_turtlebot demo_depth_camera_2d.launch bag_filename:=${HOME}/Downloads/cartographer_turtlebot_demo.bag

    # Launch the 3D depth camera demo.
    roslaunch cartographer_turtlebot demo_depth_camera_3d.launch bag_filename:=${HOME}/Downloads/cartographer_turtlebot_demo.bag

The launch files will bring up ``roscore`` and ``rviz`` automatically.
