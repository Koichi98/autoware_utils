// Copyright 2023 Tier IV, Inc.
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

// =============== Note ===============
// This is a util class implementation of the logger_config_component provided by ROS 2
// https://github.com/ros2/demos/blob/humble/logging_demo/src/logger_config_component.cpp
//
// When ROS 2 officially supports the set_logger_level option in release version, this class can be
// removed.
// https://github.com/ros2/ros2/issues/1355

// =============== How to use ===============
// ___In your_node.hpp___
// #include "autoware_utils_logging/logger_level_configure.hpp"
// class YourNode : public rclcpp::Node {
//   ...
//
//   // Define logger_configure as a node class member variable
//   std::unique_ptr<autoware_utils_logging::LoggerLevelConfigure> logger_configure_;
// }
//
// ___In your_node.cpp___
// YourNode::YourNode() {
//   ...
//
//   // Set up logger_configure
//   logger_configure_ = std::make_unique<LoggerLevelConfigure>(this);
// }
//
// For agnocast::Node, use BasicLoggerLevelConfigure<agnocast::Node>:
//   std::unique_ptr<autoware_utils_logging::BasicLoggerLevelConfigure<agnocast::Node>>
//     logger_configure_;
//   logger_configure_ =
//     std::make_unique<autoware_utils_logging::BasicLoggerLevelConfigure<agnocast::Node>>(this);

#ifndef AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_
#define AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_

#include <agnocast/agnocast.hpp>
#include <logging_demo/srv/config_logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcutils/logging.h>

#include <functional>
#include <string>
#include <type_traits>
#include <variant>

namespace autoware_utils_logging
{

using ConfigLogger = logging_demo::srv::ConfigLogger;

// Type traits to determine service type based on node type
template <typename NodeT>
struct LoggerServiceTraits
{
  using ServicePtr = typename rclcpp::Service<ConfigLogger>::SharedPtr;
};

template <>
struct LoggerServiceTraits<agnocast::Node>
{
  using ServicePtr = typename agnocast::Service<ConfigLogger>::SharedPtr;
};

// Non-template base class for LoggerLevelConfigure.
// Allows storing both rclcpp::Node and agnocast::Node variants in the same pointer.
class LoggerLevelConfigureInterface
{
public:
  virtual ~LoggerLevelConfigureInterface() = default;
};

template <typename NodeT = rclcpp::Node>
class BasicLoggerLevelConfigure : public LoggerLevelConfigureInterface
{
public:
  using ServicePtr = typename LoggerServiceTraits<NodeT>::ServicePtr;

  explicit BasicLoggerLevelConfigure(NodeT * node) : ros_logger_(node->get_logger())
  {
    if constexpr (std::is_same_v<NodeT, agnocast::Node>) {
      srv_config_logger_ = node->template create_service<ConfigLogger>(
        "~/config_logger",
        [this](
          const agnocast::ipc_shared_ptr<agnocast::Service<ConfigLogger>::RequestT> & request,
          agnocast::ipc_shared_ptr<agnocast::Service<ConfigLogger>::ResponseT> & response) {
          handle_logger_config(request->level, request->logger_name, response->success);
        });
    } else {
      srv_config_logger_ = node->template create_service<ConfigLogger>(
        "~/config_logger",
        [this](
          const ConfigLogger::Request::SharedPtr request,
          const ConfigLogger::Response::SharedPtr response) {
          handle_logger_config(request->level, request->logger_name, response->success);
        });
    }
  }

private:
  void handle_logger_config(
    const std::string & level, const std::string & logger_name, bool & success)
  {
    int logging_severity;
    const auto ret_level = rcutils_logging_severity_level_from_string(
      level.c_str(), rcl_get_default_allocator(), &logging_severity);

    if (ret_level != RCUTILS_RET_OK) {
      success = false;
      RCLCPP_WARN_STREAM(
        ros_logger_, "Failed to change logger level for "
                       << logger_name
                       << " due to an invalid logging severity: " << level);
      return;
    }

    const auto ret_set =
      rcutils_logging_set_logger_level(logger_name.c_str(), logging_severity);

    if (ret_set != RCUTILS_RET_OK) {
      success = false;
      RCLCPP_WARN_STREAM(ros_logger_, "Failed to set logger level for " << logger_name);
      return;
    }

    success = true;
    RCLCPP_INFO_STREAM(
      ros_logger_, "Logger level [" << level << "] is set for " << logger_name);
  }

  rclcpp::Logger ros_logger_;
  ServicePtr srv_config_logger_;
};

// Backward compatibility alias
using LoggerLevelConfigure = BasicLoggerLevelConfigure<rclcpp::Node>;

}  // namespace autoware_utils_logging

#endif  // AUTOWARE_UTILS_LOGGING__LOGGER_LEVEL_CONFIGURE_HPP_
