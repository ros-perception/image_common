// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "./pybind11.hpp"
#include "image_transport_py/cast_image.hpp"

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

namespace image_transport_python
{

template<typename SubscriberType>
using SubscriberMap = std::unordered_map<SubscriberType *, std::shared_ptr<SubscriberType>>;

// Bindings for the image_transport classes
PYBIND11_MODULE(_image_transport, m)
{
  m.doc() = "Python wrapper of the image_transport API";

  pybind11::class_<image_transport::Publisher>(m, "Publisher")
  .def(pybind11::init())
  .def("get_topic", &image_transport::Publisher::getTopic, "Returns the base image topic.")
  .def(
    "get_num_subscribers", &image_transport::Publisher::getNumSubscribers,
    "Returns the number of subscribers this publisher is connected to.")
  .def(
    "shutdown", &image_transport::Publisher::shutdown,
    "Unsubscribe the callback associated with this Publisher.")
  .def(
    "publish",
    [](image_transport::Publisher & publisher, sensor_msgs::msg::Image & img) {
      publisher.publish(img);
    },
    "Publish an image on the topics associated with this Publisher.");

  pybind11::class_<image_transport::CameraPublisher>(m, "CameraPublisher")
  .def(pybind11::init())
  .def(
    "get_topic", &image_transport::CameraPublisher::getTopic,
    "Returns the base (image) topic of this CameraPublisher.")
  .def(
    "get_num_subscribers", &image_transport::CameraPublisher::getNumSubscribers,
    "Returns the number of subscribers this camera publisher is connected to.")
  .def(
    "shutdown", &image_transport::CameraPublisher::shutdown,
    "Unsubscribe the callback associated with this CameraPublisher.")
  .def(
    "publish",
    [](image_transport::CameraPublisher & publisher, sensor_msgs::msg::Image & img,
    sensor_msgs::msg::CameraInfo & info) {
      publisher.publish(img, info);
    },
    "Publish an image and camera info on the topics associated with this Publisher.");

  pybind11::class_<image_transport::ImageTransport>(m, "ImageTransport")
  .def(
    pybind11::init(
      [](const std::string & node_name, const std::string & image_transport,
      const std::string & launch_params_filepath) {
        if (!rclcpp::ok()) {
          rclcpp::init(0, nullptr);
        }
        rclcpp::NodeOptions node_options;

        if (!launch_params_filepath.empty()) {
          node_options.allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true)
          .arguments({"--ros-args", "--params-file", launch_params_filepath});
        } else if (!image_transport.empty()) {
          node_options.allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true)
          .append_parameter_override("image_transport", image_transport);
        }

        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(node_name, "", node_options);

        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

        auto spin_node = [node, executor]() {
          executor->add_node(node);
          executor->spin();
        };
        std::thread execution_thread(spin_node);
        execution_thread.detach();

        return image_transport::ImageTransport(node);
      }),
    pybind11::arg("node_name"), pybind11::arg("image_transport") = "",
    pybind11::arg("launch_params_filepath") = "",

    "Initialize an ImageTransport object with a node name, image_transport"
    " and launch params file path.")
  .def(
    "advertise",
    pybind11::overload_cast<const std::string &, uint32_t, bool>(
      &image_transport::ImageTransport::advertise),
    pybind11::arg("base_topic"), pybind11::arg("queue_size"), pybind11::arg("latch") = false,
    "Advertise an image topic.")
  .def(
    "advertise_camera",
    pybind11::overload_cast<const std::string &, uint32_t, bool>(
      &image_transport::ImageTransport::advertiseCamera),
    pybind11::arg("base_topic"), pybind11::arg("queue_size"), pybind11::arg("latch") = false,
    "Advertise an image topic with camera info.")
  .def(
    "subscribe",
    [](image_transport::ImageTransport & image_transport,
    const std::string & base_topic,
    uint32_t queue_size,
    const image_transport::Subscriber::Callback & callback) {

      // Static vector to keep the subscriptions alive
      thread_local auto vec = std::vector<std::shared_ptr<image_transport::Subscriber>>();
      auto subscription =
      std::make_shared<image_transport::Subscriber>(
        image_transport.subscribe(
          base_topic,
          queue_size, callback));
      vec.push_back(subscription);

      return subscription;
    },
    pybind11::arg("base_topic"), pybind11::arg("queue_size"), pybind11::arg("callback"),
    "Subscribe to an image topic.")
  .def(
    "subscribe_camera",
    [](image_transport::ImageTransport & image_transport,
    const std::string & base_topic,
    uint32_t queue_size,
    const image_transport::CameraSubscriber::Callback & callback) {

      // Static vector to keep the subscriptions alive
      thread_local auto vec = std::vector<std::shared_ptr<image_transport::CameraSubscriber>>();
      auto subscription =
      std::make_shared<image_transport::CameraSubscriber>(
        image_transport.subscribeCamera(
          base_topic,
          queue_size, callback));
      vec.push_back(subscription);

      return subscription;
    },
    pybind11::arg("base_topic"), pybind11::arg("queue_size"), pybind11::arg("callback"),
    "Subscribe to an image topic with camera info.");

  pybind11::class_<image_transport::Subscriber, std::shared_ptr<image_transport::Subscriber>>(
    m,
    "Subscriber")
  .def(pybind11::init())
  .def(
    "get_topic",
    &image_transport::Subscriber::getTopic,
    "Returns the base image topic.")
  .def(
    "get_num_publishers",
    &image_transport::Subscriber::getNumPublishers,
    "Returns the number of publishers this subscriber is connected to.")
  .def(
    "get_transport",
    &image_transport::Subscriber::getTransport,
    "Returns the name of the transport being used.")
  .def(
    "shutdown", &image_transport::Subscriber::shutdown,
    "Unsubscribe the callback associated with this Subscriber.");

  pybind11::class_<image_transport::CameraSubscriber,
    std::shared_ptr<image_transport::CameraSubscriber>>(
    m,
    "CameraSubscriber")
  .def(pybind11::init())
  .def(
    "get_topic",
    &image_transport::CameraSubscriber::getTopic,
    "Returns the base image topic.")
  .def(
    "get_num_publishers",
    &image_transport::CameraSubscriber::getNumPublishers,
    "Returns the number of publishers this subscriber is connected to.")
  .def(
    "get_transport",
    &image_transport::CameraSubscriber::getTransport,
    "Returns the name of the transport being used.")
  .def(
    "shutdown", &image_transport::CameraSubscriber::shutdown,
    "Unsubscribe the callback associated with this CameraSubscriber.");
}
}  // namespace image_transport_python
