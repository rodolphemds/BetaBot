/**
* @file         listen_node.cpp
* @author       LDRobot (marketing1@ldrobot.com)
* @brief
* @version      0.1
* @date         2022.04.08
* @note
* @copyright    Copyright (c) 2020  SHENZHEN LDROBOT CO., LTD. All rights reserved.
* Licensed under the MIT License (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License in the file LICENSE
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <stdlib.h>
#include <string>

#define RADIAN_TO_DEGREES(angle) ((angle)*180000/3141.59)

void LidarMsgCallback(const sensor_msgs::msg::LaserScan::SharedPtr data)
{
  RCLCPP_INFO(rclcpp::get_logger("listen_node"), "[ldrobot]------listen lidar message-------");
  unsigned int lens = static_cast<unsigned int>(
    (data->angle_max - data->angle_min) / data->angle_increment);

  RCLCPP_INFO(rclcpp::get_logger("listen_node"),
    "[ldrobot] angle_min: %f angle_max: %f",
    RADIAN_TO_DEGREES(data->angle_min), RADIAN_TO_DEGREES(data->angle_max));
  RCLCPP_INFO(rclcpp::get_logger("listen_node"), "[ldrobot] point size: %zu",
    data->ranges.size());

  for (unsigned int i = 0; i < lens; i++) {
    RCLCPP_INFO(rclcpp::get_logger("listen_node"),
      "[ldrobot] angle: %f range: %f intensites: %f",
      RADIAN_TO_DEGREES((data->angle_min + i * data->angle_increment)),
      data->ranges[i], data->intensities[i]);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, "ldlidar_listen_node");
  auto node = rclcpp::Node::make_shared("ldlidar_listen_node");

  std::string topic_name;
  node->declare_parameter<std::string>("topic_name", "scan");
  node->get_parameter("topic_name", topic_name);

  if (topic_name.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "[ldrobot] [ldldiar_listen_node] input param <topic_name> is null");
    exit(EXIT_FAILURE);
  } else {
    RCLCPP_INFO(node->get_logger(),
      "[ldrobot] [ldldiar_listen_node] input param <topic_name> is %s", topic_name.c_str());
  }

  auto msg_subs = node->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_name, 10, &LidarMsgCallback);

  RCLCPP_INFO(node->get_logger(), "[ldrobot] start ldldiar message subscribe node");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF * FILE ********/
