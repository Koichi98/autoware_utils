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

#ifndef AUTOWARE_UTILS_DEBUG__DEBUG_PUBLISHER_HPP_
#define AUTOWARE_UTILS_DEBUG__DEBUG_PUBLISHER_HPP_

#include "autoware_utils_debug/debug_traits.hpp"

#include <agnocast/agnocast.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <any>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace autoware_utils_debug
{
namespace debug_publisher
{
template <
  class T_msg, class T,
  std::enable_if_t<
    autoware_utils_debug::debug_traits::is_debug_message<T_msg>::value, std::nullptr_t> = nullptr>
T_msg to_debug_msg(const T & data, const rclcpp::Time & stamp)
{
  T_msg msg;
  msg.stamp = stamp;
  msg.data = data;
  return msg;
}
}  // namespace debug_publisher

// Type traits to check if NodeT is agnocast::Node
template <typename NodeT>
struct DebugPublisherTraits
{
  static constexpr bool is_agnocast = false;
};

template <>
struct DebugPublisherTraits<agnocast::Node>
{
  static constexpr bool is_agnocast = true;
};

template <typename NodeT = rclcpp::Node>
class BasicDebugPublisher
{
public:
  static constexpr bool is_agnocast = DebugPublisherTraits<NodeT>::is_agnocast;

  explicit BasicDebugPublisher(NodeT * node, const char * ns) : node_(node), ns_(ns) {}

  template <
    class T,
    std::enable_if_t<rosidl_generator_traits::is_message<T>::value, std::nullptr_t> = nullptr>
  void publish(const std::string & name, const T & data, const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    if (pub_map_.count(name) == 0) {
      if constexpr (is_agnocast) {
        pub_map_[name] =
          node_->template create_publisher<T>(std::string(ns_) + "/" + name, qos);
      } else {
        pub_map_[name] =
          node_->template create_publisher<T>(std::string(ns_) + "/" + name, qos);
      }
    }

    if constexpr (is_agnocast) {
      auto & pub =
        std::any_cast<typename agnocast::Publisher<T>::SharedPtr &>(pub_map_.at(name));
      auto msg = pub->borrow_loaned_message();
      *msg = data;
      pub->publish(std::move(msg));
    } else {
      auto & pub = std::any_cast<typename rclcpp::Publisher<T>::SharedPtr &>(pub_map_.at(name));
      pub->publish(data);
    }
  }

  template <
    class T_msg, class T,
    std::enable_if_t<!rosidl_generator_traits::is_message<T>::value, std::nullptr_t> = nullptr>
  void publish(const std::string & name, const T & data, const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    publish(name, debug_publisher::to_debug_msg<T_msg>(data, node_->now()), qos);
  }

private:
  NodeT * node_;
  const char * ns_;
  std::unordered_map<std::string, std::any> pub_map_;
};

using DebugPublisher = BasicDebugPublisher<rclcpp::Node>;
}  // namespace autoware_utils_debug

#endif  // AUTOWARE_UTILS_DEBUG__DEBUG_PUBLISHER_HPP_
