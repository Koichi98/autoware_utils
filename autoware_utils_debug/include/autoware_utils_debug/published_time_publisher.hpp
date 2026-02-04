// Copyright 2024 The Autoware Contributors
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

#ifndef AUTOWARE_UTILS_DEBUG__PUBLISHED_TIME_PUBLISHER_HPP_
#define AUTOWARE_UTILS_DEBUG__PUBLISHED_TIME_PUBLISHER_HPP_

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_msgs/msg/published_time.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <utility>

namespace autoware_utils_debug
{

// Type traits for node-specific types
template <typename NodeT>
struct PublishedTimePublisherTraits;

template <>
struct PublishedTimePublisherTraits<rclcpp::Node>
{
  template <typename MessageT>
  using PublisherPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;

  using InputPublisherPtr = rclcpp::PublisherBase::ConstSharedPtr;
};

template <>
struct PublishedTimePublisherTraits<agnocast::Node>
{
  template <typename MessageT>
  using PublisherPtr = typename agnocast::Publisher<MessageT>::SharedPtr;

  template <typename MessageT>
  using InputPublisherPtr = typename agnocast::Publisher<MessageT>::SharedPtr;
};

template <typename NodeT = rclcpp::Node>
class BasicPublishedTimePublisher
{
public:
  using Traits = PublishedTimePublisherTraits<NodeT>;
  using PublishedTime = autoware_internal_msgs::msg::PublishedTime;
  using PublishedTimePublisherPtr = typename Traits::template PublisherPtr<PublishedTime>;

  explicit BasicPublishedTimePublisher(
    NodeT * node, std::string publisher_topic_suffix = "/debug/published_time",
    const rclcpp::QoS & qos = rclcpp::QoS(1))
  : node_(node), publisher_topic_suffix_(std::move(publisher_topic_suffix)), qos_(qos)
  {
  }

  // For rclcpp::Node - accepts rclcpp::PublisherBase
  template <typename U = NodeT>
  typename std::enable_if<std::is_same<U, rclcpp::Node>::value>::type publish_if_subscribed(
    const rclcpp::PublisherBase::ConstSharedPtr & publisher, const rclcpp::Time & stamp)
  {
    publish_if_subscribed_impl(publisher->get_gid(), publisher->get_topic_name(), stamp);
  }

  template <typename U = NodeT>
  typename std::enable_if<std::is_same<U, rclcpp::Node>::value>::type publish_if_subscribed(
    const rclcpp::PublisherBase::ConstSharedPtr & publisher, const std_msgs::msg::Header & header)
  {
    publish_if_subscribed_impl(publisher->get_gid(), publisher->get_topic_name(), header);
  }

  // For agnocast::Node - accepts agnocast::Publisher
  template <typename MessageT, typename U = NodeT>
  typename std::enable_if<std::is_same<U, agnocast::Node>::value>::type publish_if_subscribed(
    const typename agnocast::Publisher<MessageT>::SharedPtr & publisher, const rclcpp::Time & stamp)
  {
    publish_if_subscribed_impl(publisher->get_gid(), publisher->get_topic_name(), stamp);
  }

  template <typename MessageT, typename U = NodeT>
  typename std::enable_if<std::is_same<U, agnocast::Node>::value>::type publish_if_subscribed(
    const typename agnocast::Publisher<MessageT>::SharedPtr & publisher,
    const std_msgs::msg::Header & header)
  {
    publish_if_subscribed_impl(publisher->get_gid(), publisher->get_topic_name(), header);
  }

private:
  NodeT * node_;
  std::string publisher_topic_suffix_;
  rclcpp::QoS qos_;

  // Custom comparison struct for rmw_gid_t
  struct GidCompare
  {
    bool operator()(const rmw_gid_t & lhs, const rmw_gid_t & rhs) const
    {
      return std::memcmp(lhs.data, rhs.data, RMW_GID_STORAGE_SIZE) < 0;
    }
  };

  std::map<rmw_gid_t, PublishedTimePublisherPtr, GidCompare> publishers_;

  void ensure_publisher_exists(const rmw_gid_t & gid_key, const std::string & topic_name)
  {
    if (publishers_.find(gid_key) == publishers_.end()) {
      publishers_[gid_key] =
        node_->template create_publisher<PublishedTime>(topic_name + publisher_topic_suffix_, qos_);
    }
  }

  void publish_if_subscribed_impl(
    const rmw_gid_t & gid, const char * topic_name, const rclcpp::Time & stamp)
  {
    ensure_publisher_exists(gid, topic_name);

    const auto & pub_published_time = publishers_[gid];
    if (pub_published_time->get_subscription_count() > 0) {
      publish_message(pub_published_time, stamp);
    }
  }

  void publish_if_subscribed_impl(
    const rmw_gid_t & gid, const char * topic_name, const std_msgs::msg::Header & header)
  {
    ensure_publisher_exists(gid, topic_name);

    const auto & pub_published_time = publishers_[gid];
    if (pub_published_time->get_subscription_count() > 0) {
      publish_message(pub_published_time, header);
    }
  }

  // For rclcpp::Node
  template <typename U = NodeT>
  typename std::enable_if<std::is_same<U, rclcpp::Node>::value>::type publish_message(
    const PublishedTimePublisherPtr & pub, const rclcpp::Time & stamp)
  {
    PublishedTime published_time;
    published_time.header.stamp = stamp;
    published_time.published_stamp = rclcpp::Clock().now();
    pub->publish(published_time);
  }

  template <typename U = NodeT>
  typename std::enable_if<std::is_same<U, rclcpp::Node>::value>::type publish_message(
    const PublishedTimePublisherPtr & pub, const std_msgs::msg::Header & header)
  {
    PublishedTime published_time;
    published_time.header = header;
    published_time.published_stamp = rclcpp::Clock().now();
    pub->publish(published_time);
  }

  // For agnocast::Node
  template <typename U = NodeT>
  typename std::enable_if<std::is_same<U, agnocast::Node>::value>::type publish_message(
    const PublishedTimePublisherPtr & pub, const rclcpp::Time & stamp)
  {
    auto msg = pub->borrow_loaned_message();
    msg->header.stamp = stamp;
    msg->published_stamp = rclcpp::Clock().now();
    pub->publish(std::move(msg));
  }

  template <typename U = NodeT>
  typename std::enable_if<std::is_same<U, agnocast::Node>::value>::type publish_message(
    const PublishedTimePublisherPtr & pub, const std_msgs::msg::Header & header)
  {
    auto msg = pub->borrow_loaned_message();
    msg->header = header;
    msg->published_stamp = rclcpp::Clock().now();
    pub->publish(std::move(msg));
  }
};

using PublishedTimePublisher = BasicPublishedTimePublisher<rclcpp::Node>;

}  // namespace autoware_utils_debug

#endif  // AUTOWARE_UTILS_DEBUG__PUBLISHED_TIME_PUBLISHER_HPP_
