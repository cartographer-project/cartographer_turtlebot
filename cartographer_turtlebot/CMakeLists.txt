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

cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(cartographer_turtlebot)

set(PACKAGE_DEPENDENCIES
  cartographer_ros
  roscpp
  roslib
  sensor_msgs
)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()
google_enable_testing()

find_package(LuaGoogle REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# Override Catkin's GTest configuration to use GMock.
set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

catkin_package(CATKIN_DEPENDS message_runtime ${PACKAGE_DEPENDENCIES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(configuration_files_test "cartographer_turtlebot/configuration_files_test.cc")
  # catkin_add_gtest uses a plain (i.e. no PUBLIC/PRIVATE/INTERFACE) call to
  # target_link_libraries. That forces us to do the same.
  target_include_directories(configuration_files_test SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
  target_link_libraries(configuration_files_test ${LUA_LIBRARIES})
  target_include_directories(configuration_files_test SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(configuration_files_test ${catkin_LIBRARIES})
  add_dependencies(configuration_files_test ${catkin_EXPORTED_TARGETS})
  target_include_directories(configuration_files_test SYSTEM PUBLIC ${CARTOGRAPHER_INCLUDE_DIRS})
  target_link_libraries(configuration_files_test ${CARTOGRAPHER_LIBRARIES})
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
  set_target_properties(configuration_files_test PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
endif()

add_executable(cartographer_flat_world_imu_node "cartographer_turtlebot/flat_world_imu_node_main.cc")
target_include_directories(cartographer_flat_world_imu_node SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(cartographer_flat_world_imu_node ${catkin_LIBRARIES})
add_dependencies(cartographer_flat_world_imu_node ${catkin_EXPORTED_TARGETS})
set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(cartographer_flat_world_imu_node PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS cartographer_flat_world_imu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
