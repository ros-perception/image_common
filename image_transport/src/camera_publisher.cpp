// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "image_transport/camera_publisher.hpp"

#include <memory>
#include <string>

#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

#include "image_transport/camera_common.hpp"
#include "image_transport/image_transport.hpp"

namespace image_transport
{

struct CameraPublisher::Impl
{
  explicit Impl(rclcpp::Node::SharedPtr node)
  : node_(node),
    logger_(node->get_logger()),
    unadvertised_(false)
  {
  }

  explicit Impl(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : lifecycle_node_(node),
    logger_(node->get_logger()),
    unadvertised_(false)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !unadvertised_;
  }

  void shutdown()
  {
    if (!unadvertised_) {
      unadvertised_ = true;
      image_pub_.shutdown();
      info_pub_.reset();
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  rclcpp::Logger logger_;
  Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  bool unadvertised_;
};

CameraPublisher::CameraPublisher(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions pub_options)
: impl_(std::make_shared<Impl>(node))
{
  CameraPublisher(base_topic, custom_qos, pub_options);
}

CameraPublisher::CameraPublisher(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions pub_options)
: impl_(std::make_shared<Impl>(node))
{
  CameraPublisher(base_topic, custom_qos, pub_options);
}

CameraPublisher::CameraPublisher(
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions pub_options)
{
  if (!impl_)
  {
    throw std::runtime_error("impl is not constructed!");
  }
  // Explicitly resolve name here so we compute the correct CameraInfo topic when the
  // image topic is remapped (#4539).
  std::string image_topic;
  if (impl_->node_) {
    image_topic = rclcpp::expand_topic_or_service_name(
      base_topic,
      impl_->node_->get_name(), impl_->node_->get_namespace());
  } else {
    image_topic = rclcpp::expand_topic_or_service_name(
      base_topic,
      impl_->lifecycle_node_->get_name(), impl_->lifecycle_node_->get_namespace());
  }
  std::string info_topic = getCameraInfoTopic(image_topic);

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos);
  if (impl_->node_) {
    impl_->image_pub_ = image_transport::create_publisher(impl_->node_, image_topic, custom_qos, pub_options);
    impl_->info_pub_ = impl_->node_->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, qos);
  } else {
    impl_->image_pub_ = image_transport::create_publisher(impl_->lifecycle_node_, image_topic, custom_qos, pub_options);
    impl_->info_pub_ = impl_->lifecycle_node_->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, qos);
  }
}

size_t CameraPublisher::getNumSubscribers() const
{
  if (impl_ && impl_->isValid()) {
    return std::max(
      impl_->image_pub_.getNumSubscribers(),
      impl_->info_pub_->get_subscription_count());
  }
  return 0;
}

std::string CameraPublisher::getTopic() const
{
  if (impl_) {
    return impl_->image_pub_.getTopic();
  }
  return std::string();
}

std::string CameraPublisher::getInfoTopic() const
{
  if (impl_) {
    return impl_->info_pub_->get_topic_name();
  }
  return std::string();
}

void CameraPublisher::publish(
  const sensor_msgs::msg::Image & image,
  const sensor_msgs::msg::CameraInfo & info) const
{
  if (!impl_ || !impl_->isValid()) {
    // TODO(ros2) Switch to RCUTILS_ASSERT when ros2/rcutils#112 is merged
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(
      logger,
      "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }

  impl_->image_pub_.publish(image);
  impl_->info_pub_->publish(info);
}

void CameraPublisher::publish(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info) const
{
  if (!impl_ || !impl_->isValid()) {
    // TODO(ros2) Switch to RCUTILS_ASSERT when ros2/rcutils#112 is merged
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(
      logger,
      "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }

  impl_->image_pub_.publish(*image);
  impl_->info_pub_->publish(*info);
}

void CameraPublisher::publish(
  sensor_msgs::msg::Image & image, sensor_msgs::msg::CameraInfo & info,
  rclcpp::Time stamp) const
{
  if (!impl_ || !impl_->isValid()) {
    // TODO(ros2) Switch to RCUTILS_ASSERT when ros2/rcutils#112 is merged
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(
      logger,
      "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }

  image.header.stamp = stamp;
  info.header.stamp = stamp;
  impl_->image_pub_.publish(image);
  impl_->info_pub_->publish(info);
}

void CameraPublisher::shutdown()
{
  if (impl_) {
    impl_->shutdown();
    impl_.reset();
  }
}

CameraPublisher::operator void *() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void *>(1) : reinterpret_cast<void *>(0);
}

}  // namespace image_transport
