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

namespace image_transport_python
{
// Bindings for the ImageCodec class
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

  pybind11::class_<image_transport::ImageTransport>(m, "ImageTransport")
  .def(
    pybind11::init(
      [](const std::string & node_name, const std::string & launch_params_filepath) {
        if (!rclcpp::ok()) {
          rclcpp::init(0, nullptr);
        }
        rclcpp::NodeOptions node_options;

        if (!launch_params_filepath.empty()) {
          node_options.allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true)
          .arguments({"--ros-args", "--params-file", launch_params_filepath});
        }
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(node_name, "", node_options);

        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        auto spin_node = [node, executor]() {
        executor->add_node(node);
        executor->spin();
      };
        std::thread execution_thread(spin_node);
        execution_thread.detach();

        return image_transport::ImageTransport(node);
      }))
  .def(
    "advertise",
    pybind11::overload_cast<const std::string &, uint32_t, bool>(
      &image_transport::ImageTransport::advertise),
    pybind11::arg("base_topic"), pybind11::arg("queue_size"), pybind11::arg("latch") = false);
}
}  // namespace image_transport_python
