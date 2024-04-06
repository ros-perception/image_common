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

#include "image_transport/camera_subscriber.hpp"

#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "image_transport/camera_common.hpp"
#include "image_transport/subscriber_filter.hpp"

inline void increment(int * value)
{
  ++(*value);
}

namespace image_transport
{

struct CameraSubscriber::Impl
{
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using TimeSync = message_filters::TimeSynchronizer<Image, CameraInfo>;

  explicit Impl(rclcpp::Node::SharedPtr node)
  : node_(node),
    logger_(node->get_logger()),
    sync_(10),
    unsubscribed_(false),
    image_received_(0), info_received_(0), both_received_(0)
  {
  }

  explicit Impl(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : lifecycle_node_(node),
    logger_(node->get_logger()),
    sync_(10),
    unsubscribed_(false),
    image_received_(0), info_received_(0), both_received_(0)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !unsubscribed_;
  }

  void shutdown()
  {
    if (!unsubscribed_) {
      unsubscribed_ = true;
      image_sub_.unsubscribe();
      if (node_) {
        info_sub_.unsubscribe();
      } else {
        lifecycle_info_sub_.unsubscribe();
      }
    }
  }

  void checkImagesSynchronized()
  {
    int threshold = 3 * both_received_;
    if (image_received_ > threshold || info_received_ > threshold) {
      std::string info_topic;
      if (node_) {
        info_topic = info_sub_.getTopic();
      } else {
        info_topic = lifecycle_info_sub_.getTopic();
      }
      RCLCPP_WARN(
        logger_,
        "[image_transport] Topics '%s' and '%s' do not appear to be synchronized. "
        "In the last 10s:\n"
        "\tImage messages received:      %d\n"
        "\tCameraInfo messages received: %d\n"
        "\tSynchronized pairs:           %d",
        image_sub_.getTopic().c_str(), info_topic.c_str(),
        image_received_, info_received_, both_received_);
    }
    image_received_ = info_received_ = both_received_ = 0;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  rclcpp::Logger logger_;
  SubscriberFilter image_sub_;
  message_filters::Subscriber<CameraInfo> info_sub_;
  message_filters::Subscriber<CameraInfo, rclcpp_lifecycle::LifecycleNode> lifecycle_info_sub_;
  TimeSync sync_;

  bool unsubscribed_;
  // For detecting when the topics aren't synchronized
  std::shared_ptr<rclcpp::TimerBase> check_synced_timer_;
  int image_received_, info_received_, both_received_;
};

CameraSubscriber::CameraSubscriber(
  rclcpp::Node::SharedPtr node,
  const std::string & base_topic,
  const Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos)
: impl_(std::make_shared<Impl>(node))
{
  CameraSubscriber(base_topic, callback, transport, custom_qos);
}

CameraSubscriber::CameraSubscriber(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & base_topic,
  const Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos)
: impl_(std::make_shared<Impl>(node))
{
  CameraSubscriber(base_topic, callback, transport, custom_qos);
}

CameraSubscriber::CameraSubscriber(
  const std::string & base_topic,
  const Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos)
{
  if (!impl_)
  {
    throw std::runtime_error("impl is not constructed!");
  }
  // Must explicitly remap the image topic since we then do some string manipulation on it
  // to figure out the sibling camera_info topic.
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

  if (impl_->node_) {
    impl_->image_sub_.subscribe(impl_->node_, image_topic, transport, custom_qos);
    impl_->info_sub_.subscribe(impl_->node_, info_topic, custom_qos);
    impl_->sync_.connectInput(impl_->image_sub_, impl_->info_sub_);
    impl_->info_sub_.registerCallback(std::bind(increment, &impl_->info_received_));
  } else {
    impl_->image_sub_.subscribe(impl_->lifecycle_node_, image_topic, transport, custom_qos);
    impl_->lifecycle_info_sub_.subscribe(impl_->lifecycle_node_, info_topic, custom_qos);
    impl_->sync_.connectInput(impl_->image_sub_, impl_->lifecycle_info_sub_);
    impl_->lifecycle_info_sub_.registerCallback(std::bind(increment, &impl_->info_received_));
  }

  impl_->sync_.registerCallback(std::bind(callback, std::placeholders::_1, std::placeholders::_2));

  // Complain every 10s if it appears that the image and info topics are not synchronized
  impl_->image_sub_.registerCallback(std::bind(increment, &impl_->image_received_));
  impl_->sync_.registerCallback(std::bind(increment, &impl_->both_received_));

  if (impl_->node_) {
    impl_->check_synced_timer_ = impl_->node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Impl::checkImagesSynchronized, impl_.get()));
  } else {
    impl_->check_synced_timer_ = impl_->lifecycle_node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Impl::checkImagesSynchronized, impl_.get()));
  }
}

std::string CameraSubscriber::getTopic() const
{
  if (impl_) {return impl_->image_sub_.getTopic();}
  return std::string();
}

std::string CameraSubscriber::getInfoTopic() const
{
  if (impl_) {
    if (impl_->node_) {
      return impl_->info_sub_.getSubscriber()->get_topic_name();
    } else {
      return impl_->lifecycle_info_sub_.getSubscriber()->get_topic_name();
    }
  }
  return std::string();
}

size_t CameraSubscriber::getNumPublishers() const
{
  if (impl_) {
    size_t info_pub_count;
    if (impl_->node_) {
      info_pub_count = impl_->info_sub_.getSubscriber()->get_publisher_count();
    } else {
      info_pub_count = impl_->lifecycle_info_sub_.getSubscriber()->get_publisher_count();
    }

    return std::max(
      impl_->image_sub_.getSubscriber().getNumPublishers(),
      info_pub_count);
  }
  return 0;
}

std::string CameraSubscriber::getTransport() const
{
  if (impl_) {return impl_->image_sub_.getTransport();}
  return std::string();
}

void CameraSubscriber::shutdown()
{
  if (impl_) {impl_->shutdown();}
}

CameraSubscriber::operator void *() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void *>(1) : reinterpret_cast<void *>(0);
}

}  // namespace image_transport
