// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_
#define AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_

#include <agnocast/node/agnocast_node.hpp>
#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware_utils_tf
{
class TransformListener
{
public:
  // Constructor for rclcpp::Node
  explicit TransformListener(rclcpp::Node * node)
  : clock_(node->get_clock()), logger_(node->get_logger())
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Constructor for agnocast::Node
  explicit TransformListener(agnocast::Node * node)
  : clock_(node->get_clock()), logger_(node->get_logger())
  {
    agnocast_tf_buffer_ = std::make_shared<agnocast::Buffer>(clock_);
    agnocast_tf_listener_ =
      std::make_unique<agnocast::TransformListener>(*agnocast_tf_buffer_, *node);
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr get_latest_transform(
    const std::string & from, const std::string & to)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      if (tf_buffer_) {
        tf = tf_buffer_->lookupTransform(from, to, tf2::TimePointZero);
      } else {
        tf = agnocast_tf_buffer_->lookupTransform(from, to, tf2::TimePointZero, tf2::Duration(0));
      }
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "failed to get transform from %s to %s: %s", from.c_str(),
        to.c_str(), ex.what());
      return {};
    }

    return std::make_shared<const geometry_msgs::msg::TransformStamped>(tf);
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr get_transform(
    const std::string & from, const std::string & to, const rclcpp::Time & time,
    const rclcpp::Duration & duration)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      if (tf_buffer_) {
        tf = tf_buffer_->lookupTransform(from, to, time, duration);
      } else {
        tf = agnocast_tf_buffer_->lookupTransform(
          from, to, tf2::TimePoint(std::chrono::nanoseconds(time.nanoseconds())),
          tf2::Duration(std::chrono::nanoseconds(duration.nanoseconds())));
      }
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "failed to get transform from %s to %s: %s", from.c_str(),
        to.c_str(), ex.what());
      return {};
    }

    return std::make_shared<const geometry_msgs::msg::TransformStamped>(tf);
  }

  rclcpp::Logger get_logger() { return logger_; }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  // For rclcpp::Node
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // For agnocast::Node
  std::shared_ptr<agnocast::Buffer> agnocast_tf_buffer_;
  std::unique_ptr<agnocast::TransformListener> agnocast_tf_listener_;
};
}  // namespace autoware_utils_tf

#endif  // AUTOWARE_UTILS_TF__TRANSFORM_LISTENER_HPP_
