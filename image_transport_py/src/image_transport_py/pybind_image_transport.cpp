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

class PyImageTransport : public image_transport::ImageTransport {
public:
    // Constructor that takes an rclcpp::Node::SharedPtr
    PyImageTransport(const rclcpp::Node::SharedPtr& node)
        : image_transport::ImageTransport(node) {}

    // Public member variable for the subscriber
    thread_local image_transport::Subscriber subscriber_;
};


void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      "image_transport"), "message arrived yoho. %s", msg->encoding.c_str());

}
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

  // pybind11::class_<image_transport::ImageTransport>(m, "ImageTransport")
  pybind11::class_<image_transport_python::PyImageTransport>(m, "ImageTransport")
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


        RCLCPP_INFO(node->get_logger(), "starting thread");
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

        auto spin_node = [node, executor]() {
        // auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
        //   "camera/image", 10, image_transport_python::imageCallback);

        RCLCPP_INFO(node->get_logger(), "spin on!");
        executor->add_node(node);
        executor->spin();
        // while (rclcpp::ok()) {
        // Spin some to process available callbacks, but do not block indefinitely
        // executor->spin_some();  // Spin for up to 100ms at a time
        // std::this_thread::sleep_for(1000ms);  // Simulate other tasks or waiting
        // RCLCPP_INFO(node->get_logger(), "spinnn!");
        // }

        RCLCPP_INFO(node->get_logger(), "spin off!");
      };
        std::thread execution_thread(spin_node);
        execution_thread.detach();

        // return image_transport::ImageTransport(node);
        return image_transport_python::PyImageTransport(node);
      }))
  .def(
    "advertise",
    pybind11::overload_cast<const std::string &, uint32_t, bool>(
      &image_transport::ImageTransport::advertise),
    pybind11::arg("base_topic"), pybind11::arg("queue_size"), pybind11::arg("latch") = false)
  .def(
    "advertise_camera",
    pybind11::overload_cast<const std::string &, uint32_t, bool>(
      &image_transport::ImageTransport::advertiseCamera),
    pybind11::arg("base_topic"), pybind11::arg("queue_size"), pybind11::arg("latch") = false)
  .def(
    "subscribe",
    [](image_transport_python::PyImageTransport & image_transport,
    const std::string & base_topic,
    uint32_t queue_size,
    const std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &)> & callback) {

      RCLCPP_INFO(image_transport.impl_->node_->get_logger(), "in subscribe");

      // image_transport.subscriber_ = std::make_unique<image_transport::Subscriber>(image_transport.subscribe(base_topic, queue_size, image_transport_python::imageCallback)) ;
      // auto subscription = 
      thread_local auto  subscription = std::make_shared<image_transport::Subscriber>(image_transport.subscribe(base_topic, queue_size, image_transport_python::imageCallback)) ;
      // RCLCPP_INFO(image_transport.impl_->node_->get_logger(), "subscribed to %s", subscription.getTopic().c_str());

      return subscription;

  },  
    pybind11::arg("base_topic"), pybind11::arg("queue_size"), pybind11::arg("callback"))

  ;

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


}
}  // namespace image_transport_python
