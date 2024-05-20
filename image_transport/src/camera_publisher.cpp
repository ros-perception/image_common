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
#include <utility>

#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

#include "image_transport/camera_common.hpp"
#include "image_transport/image_transport.hpp"

namespace image_transport
{

template<class NodeType>
struct CameraPublisher<NodeType>::Impl
{
  explicit Impl(NodeType * node)
  : node_(node),
    logger_(node->get_logger()),
    unadvertised_(false)
  {
  }

  explicit Impl(std::shared_ptr<NodeType> node)
  : node_(node),
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

  std::shared_ptr<NodeType> node_;
  rclcpp::Logger logger_;
  Publisher<NodeType> image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  bool unadvertised_;
};

template<class NodeType>
CameraPublisher<NodeType>::CameraPublisher(
  NodeType * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions pub_options)
: impl_(std::make_shared<Impl>(node))
{
  initialise(base_topic, custom_qos, pub_options);
}

template<class NodeType>
CameraPublisher<NodeType>::CameraPublisher(
  std::shared_ptr<NodeType> node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions pub_options)
: impl_(std::make_shared<Impl>(node))
{
  initialise(base_topic, custom_qos, pub_options);
}

template<class NodeType>
void CameraPublisher<NodeType>::initialise(
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions pub_options)
{
  if (!impl_) {
    throw std::runtime_error("impl is not constructed!");
  }
  // Explicitly resolve name here so we compute the correct CameraInfo topic when the
  // image topic is remapped (#4539).
  std::string image_topic;
  image_topic = rclcpp::expand_topic_or_service_name(
    base_topic,
    impl_->node_->get_name(), impl_->node_->get_namespace());
  std::string info_topic = getCameraInfoTopic(image_topic);

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos);
  impl_->image_pub_ = image_transport::create_publisher(
    impl_->node_, image_topic, custom_qos, pub_options);
  impl_->info_pub_ =
    impl_->node_->template create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, qos);
}

template<class NodeType>
size_t CameraPublisher<NodeType>::getNumSubscribers() const
{
  if (impl_ && impl_->isValid()) {
    return std::max(
      impl_->image_pub_.getNumSubscribers(),
      impl_->info_pub_->get_subscription_count());
  }
  return 0;
}

template<class NodeType>
std::string CameraPublisher<NodeType>::getTopic() const
{
  if (impl_) {
    return impl_->image_pub_.getTopic();
  }
  return std::string();
}

template<class NodeType>
std::string CameraPublisher<NodeType>::getInfoTopic() const
{
  if (impl_) {
    return impl_->info_pub_->get_topic_name();
  }
  return std::string();
}

template<class NodeType>
void CameraPublisher<NodeType>::publish(
  const sensor_msgs::msg::Image & image,
  const sensor_msgs::msg::CameraInfo & info) const
{
  if (!impl_ || !impl_->isValid()) {
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(
      logger,
      "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }

  impl_->image_pub_.publish(image);
  impl_->info_pub_->publish(info);
}

template<class NodeType>
void CameraPublisher<NodeType>::publish(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info) const
{
  if (!impl_ || !impl_->isValid()) {
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(
      logger,
      "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }

  impl_->image_pub_.publish(*image);
  impl_->info_pub_->publish(*info);
}

template<class NodeType>
void CameraPublisher<NodeType>::publish(
  sensor_msgs::msg::Image::UniquePtr image,
  sensor_msgs::msg::CameraInfo::UniquePtr info) const
{
  if (!impl_ || !impl_->isValid()) {
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(
      logger,
      "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }

  impl_->image_pub_.publish(std::move(image));
  impl_->info_pub_->publish(std::move(info));
}

template<class NodeType>
void CameraPublisher<NodeType>::publish(
  sensor_msgs::msg::Image & image, sensor_msgs::msg::CameraInfo & info,
  rclcpp::Time stamp) const
{
  if (!impl_ || !impl_->isValid()) {
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

template<class NodeType>
void CameraPublisher<NodeType>::publish(
  sensor_msgs::msg::Image::UniquePtr image,
  sensor_msgs::msg::CameraInfo::UniquePtr info,
  rclcpp::Time stamp) const
{
  if (!impl_ || !impl_->isValid()) {
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(
      logger,
      "Call to publish() on an invalid image_transport::CameraPublisher");
    return;
  }

  image->header.stamp = stamp;
  info->header.stamp = stamp;
  impl_->image_pub_.publish(std::move(image));
  impl_->info_pub_->publish(std::move(info));
}

template<class NodeType>
void CameraPublisher<NodeType>::shutdown()
{
  if (impl_) {
    impl_->shutdown();
    impl_.reset();
  }
}

template<class NodeType>
CameraPublisher<NodeType>::operator void *() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void *>(1) : reinterpret_cast<void *>(0);
}

}  // namespace image_transport

template class image_transport::CameraPublisher<rclcpp::Node>;
template class image_transport::CameraPublisher<rclcpp_lifecycle::LifecycleNode>;
