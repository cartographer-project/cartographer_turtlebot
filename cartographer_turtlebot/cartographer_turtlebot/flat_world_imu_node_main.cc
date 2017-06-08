/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

namespace {

constexpr double kFakeGravity = 9.8;
constexpr int kSubscriberQueueSize = 150;
constexpr char kImuInTopic[] = "imu_in";
constexpr char kImuOutTopic[] = "imu_out";

}  // namespace

int main(int argc, char** argv) {
  ::ros::init(argc, argv, "flat_world_imu_node");

  ::ros::NodeHandle node_handle;
  ::ros::Publisher publisher =
      node_handle.advertise<sensor_msgs::Imu>(kImuOutTopic, 10);

  ::ros::Time last_published_time;
  ::ros::Subscriber subscriber = node_handle.subscribe(
      kImuInTopic, kSubscriberQueueSize,
      boost::function<void(const sensor_msgs::Imu::ConstPtr& imu_in)>(
          [&](const sensor_msgs::Imu::ConstPtr& imu_in) {
            // The 'imu_data_raw' topic of the Kobuki base will at times publish
            // IMU messages out of order. These out of order messages must be
            // dropped.
            if (last_published_time.isZero() ||
                imu_in->header.stamp > last_published_time) {
              last_published_time = imu_in->header.stamp;
              sensor_msgs::Imu imu_out = *imu_in;
              // TODO(damonkohler): This relies on the z-axis alignment of the
              // IMU with the Kobuki base.
              imu_out.linear_acceleration.x = 0.;
              imu_out.linear_acceleration.y = 0.;
              imu_out.linear_acceleration.z = kFakeGravity;
              publisher.publish(imu_out);
            }
          }));

  ::ros::start();
  ::ros::spin();
  ::ros::shutdown();
}
