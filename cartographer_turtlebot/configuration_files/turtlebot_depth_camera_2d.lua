-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  sensor_bridge = {
    horizontal_laser_min_range = 0.1,
    horizontal_laser_max_range = 4.,
    horizontal_laser_missing_echo_ray_length = 2.,
    use_constant_odometry_variance = true,
    constant_odometry_translational_variance = 1e-7,
    constant_odometry_rotational_variance = 1e-7,
  },
  map_frame = "map",
  tracking_frame = "gyro_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry_data = true,
  use_horizontal_laser = true,
  use_horizontal_multi_echo_laser = false,
  num_lasers_3d = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.25
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.covariance_scale = 1e-2
TRAJECTORY_BUILDER_2D.submaps.num_laser_fans = 300
SPARSE_POSE_GRAPH.optimize_every_n_scans = 300
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.8

return options
