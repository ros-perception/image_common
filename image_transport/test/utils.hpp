// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include <rclcpp/event.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

namespace test_rclcpp
{
using NodeBaseInterface = rclcpp::node_interfaces::NodeBaseInterface;
using NodeGraphInterface = rclcpp::node_interfaces::NodeGraphInterface;
using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;
using NodeParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;
using NodeTimersInterface = rclcpp::node_interfaces::NodeTimersInterface;
using NodeTopicsInterface = rclcpp::node_interfaces::NodeTopicsInterface;

using RequiredInterfacesTest = rclcpp::node_interfaces::NodeInterfaces<
  NodeBaseInterface,
  NodeGraphInterface,
  NodeLoggingInterface,
  NodeParametersInterface,
  NodeTimersInterface,
  NodeTopicsInterface>;

// Sleep for timeout ms or until a subscriber has registered for the topic
// if to_be_available is true, then it will wait for the number of
// subscribers to be > 0, if false it will wait for the number of
// subscribers to be 0
void wait_for_subscriber(
  RequiredInterfacesTest node_interfaces,
  const std::string & topic_name,
  bool to_be_available = true,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1),
  std::chrono::microseconds sleep_period = std::chrono::seconds(1))
{
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  using std::chrono::steady_clock;
  auto start = steady_clock::now();
  microseconds time_slept(0);
  auto predicate = [&node_interfaces, &topic_name, &to_be_available]() -> bool {
      if (to_be_available) {
        // the subscriber is available if the count is gt 0
        return node_interfaces.get_node_graph_interface()->count_subscribers(topic_name) > 0;
      } else {
        // the node_interfaces is no longer available when the count is 0
        return node_interfaces.get_node_graph_interface()->count_subscribers(topic_name) == 0;
      }
    };
  while (!predicate() && time_slept < duration_cast<microseconds>(timeout)) {
    rclcpp::Event::SharedPtr graph_event =
      node_interfaces.get_node_graph_interface()->get_graph_event();
    node_interfaces.get_node_graph_interface()->wait_for_graph_change(graph_event, sleep_period);
    time_slept = duration_cast<std::chrono::microseconds>(steady_clock::now() - start);
  }
  int64_t time_slept_count =
    std::chrono::duration_cast<std::chrono::microseconds>(time_slept).count();
  printf(
    "Waited %" PRId64 " microseconds for the subscriber to %s topic '%s'\n",
    time_slept_count,
    to_be_available ? "connect to" : "disconnect from",
    topic_name.c_str());
}

// The RMW may update the ROS graph asynchronously (e.g. rmw_cyclonedds adds
// local endpoints from a builtin-topic listener), so a graph query made right
// after creating or destroying an entity can be stale. Poll until the
// predicate holds or the timeout expires.
template<typename PredicateT>
bool wait_for_graph(
  PredicateT && predicate,
  std::chrono::milliseconds timeout = std::chrono::seconds(5))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

template<typename NodeT>
bool wait_for_publisher_count(
  const NodeT & node, const std::string & topic_name, size_t count)
{
  auto graph = node->get_node_graph_interface();
  return wait_for_graph(
    [&graph, &topic_name, count]() {return graph->count_publishers(topic_name) == count;});
}

template<typename NodeT>
bool wait_for_subscriber_count(
  const NodeT & node, const std::string & topic_name, size_t count)
{
  auto graph = node->get_node_graph_interface();
  return wait_for_graph(
    [&graph, &topic_name, count]() {return graph->count_subscribers(topic_name) == count;});
}

// Wait until topic_name has exactly one publisher and it belongs to node,
// then return its endpoint info.
template<typename NodeT>
std::optional<rclcpp::TopicEndpointInfo> wait_for_publisher_info(
  const NodeT & node, const std::string & topic_name)
{
  auto graph = node->get_node_graph_interface();
  std::optional<rclcpp::TopicEndpointInfo> info;
  wait_for_graph(
    [&]() {
      auto infos = graph->get_publishers_info_by_topic(topic_name);
      if (infos.size() == 1 && infos[0].node_name() == node->get_name()) {
        info = infos[0];
        return true;
      }
      return false;
    });
  return info;
}

// Wait until topic_name has exactly one subscription and it belongs to node,
// then return its endpoint info.
template<typename NodeT>
std::optional<rclcpp::TopicEndpointInfo> wait_for_subscription_info(
  const NodeT & node, const std::string & topic_name)
{
  auto graph = node->get_node_graph_interface();
  std::optional<rclcpp::TopicEndpointInfo> info;
  wait_for_graph(
    [&]() {
      auto infos = graph->get_subscriptions_info_by_topic(topic_name);
      if (infos.size() == 1 && infos[0].node_name() == node->get_name()) {
        info = infos[0];
        return true;
      }
      return false;
    });
  return info;
}

}  // namespace test_rclcpp

#endif  // UTILS_HPP_
