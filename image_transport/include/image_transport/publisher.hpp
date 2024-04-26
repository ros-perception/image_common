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

#ifndef IMAGE_TRANSPORT__PUBLISHER_HPP_
#define IMAGE_TRANSPORT__PUBLISHER_HPP_

#include <string>
#include <memory>
#include <utility>

#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "image_transport/exception.hpp"
#include "image_transport/loader_fwds.hpp"
#include "image_transport/single_subscriber_publisher.hpp"
#include "image_transport/visibility_control.hpp"
#include "image_transport/plugin_publisher.hpp"

namespace image_transport
{

/**
 * \brief Manages advertisements of multiple transport options on an Image topic.
 *
 * Publisher is a drop-in replacement for ros::Publisher when publishing
 * Image topics. In a minimally built environment, they behave the same; however,
 * Publisher is extensible via plugins to publish alternative representations of
 * the image on related subtopics. This is especially useful for limiting bandwidth and
 * latency over a network connection, when you might (for example) use the theora plugin
 * to transport the images as streamed video. All topics are published only on demand
 * (i.e. if there are subscribers).
 *
 * A Publisher should always be created through a call to ImageTransport::advertise(),
 * or copied from one that was.
 * Once all copies of a specific Publisher go out of scope, any subscriber callbacks
 * associated with that handle will stop being called. Once all Publisher for a
 * given base topic go out of scope the topic (and all subtopics) will be unadvertised.
 */
template<typename MessageT = sensor_msgs::msg::Image, typename AllocatorT = std::allocator<void>>
class PublisherBase
{
public:
  using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;

  using PublishedTypeAllocatorTraits = rclcpp::allocator::AllocRebind<PublishedType, AllocatorT>;
  using PublishedTypeAllocator = typename PublishedTypeAllocatorTraits::allocator_type;
  using PublishedTypeDeleter = rclcpp::allocator::Deleter<PublishedTypeAllocator, PublishedType>;

  using ROSMessageTypeAllocatorTraits = rclcpp::allocator::AllocRebind<ROSMessageType, AllocatorT>;
  using ROSMessageTypeAllocator = typename ROSMessageTypeAllocatorTraits::allocator_type;
  using ROSMessageTypeDeleter = rclcpp::allocator::Deleter<ROSMessageTypeAllocator, ROSMessageType>;

  static_assert(
    std::is_same_v<ROSMessageType, sensor_msgs::msg::Image>,
    "Ros Message Type must be sensor_msgs::msg::Image");

  RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase<MessageT, AllocatorT>)

  IMAGE_TRANSPORT_PUBLIC
  PublisherBase() = default;

  IMAGE_TRANSPORT_PUBLIC
  PublisherBase(
    rclcpp::Node * nh,
    const std::string & base_topic,
    PubLoaderPtr loader,
    rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions options = rclcpp::PublisherOptions())
  // Resolve the name explicitly because otherwise the compressed topics don't remap
  // properly (#3652).
  : image_topic_(rclcpp::expand_topic_or_service_name(base_topic, nh->get_name(),
      nh->get_namespace()))
    , plugin_publisher(nh, image_topic_, std::move(loader), custom_qos, options)
  {
    RCLCPP_DEBUG(nh->get_logger(), "getTopicToAdvertise: %s", image_topic_.c_str());
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos);
    raw_publisher = nh->create_publisher<MessageT>(image_topic_, qos, options);
  }

  /*!
   * \brief Returns the number of subscribers that are currently connected to
   * this Publisher.
   *
   * Returns the total number of subscribers to all advertised topics.
   */
  IMAGE_TRANSPORT_PUBLIC
  size_t getNumSubscribers() const
  {
    return raw_publisher->get_subscription_count() + plugin_publisher.getNumSubscribers();
  }

  /*!
   * \brief Returns the base topic of this Publisher.
   */
  IMAGE_TRANSPORT_PUBLIC
  std::string getTopic() const
  {
    return image_topic_;
  }

  /*!
   * \brief Publish an image on the topics associated with this Publisher.
   */
  template<class T = PublishedType>
  typename std::enable_if_t<
    std::is_same<T, sensor_msgs::msg::Image>::value
  >
  IMAGE_TRANSPORT_PUBLIC
  publish(const sensor_msgs::msg::Image & message) const
  {
    plugin_publisher.publish(message);
    publish_raw(message);
  }

  /*!
   * \brief Publish an image on the topics associated with this Publisher.
   */
  template<class T = PublishedType>
  typename std::enable_if_t<
    std::is_same<T, sensor_msgs::msg::Image>::value
  >
  IMAGE_TRANSPORT_PUBLIC
  publish(const sensor_msgs::msg::Image::ConstSharedPtr & message) const
  {
    plugin_publisher.publish(message);
    publish_raw(*message);
  }

  /*!
   * \brief Publish a type adapted image on the topics associated with this Publisher.
   */
  template<typename T>
  typename std::enable_if_t<rclcpp::TypeAdapter<MessageT>::is_specialized::value &&
    std::is_same<T, PublishedType>::value>
  IMAGE_TRANSPORT_PUBLIC publish(std::unique_ptr<T, PublishedTypeDeleter> msg) const
  {
    if (HasPluginSubscribers()) {
      auto shared_msg = std::make_shared<sensor_msgs::msg::Image>();
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(*msg, *shared_msg);

      plugin_publisher.publish(shared_msg);
    }

    publish_raw(std::move(msg));
  }

  template<typename T>
  typename std::enable_if_t<rclcpp::TypeAdapter<MessageT>::is_specialized::value &&
    std::is_same<T, PublishedType>::value>
  IMAGE_TRANSPORT_PUBLIC publish(const T & msg) const
  {
    if (HasPluginSubscribers()) {
      auto shared_msg = std::make_shared<sensor_msgs::msg::Image>();
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(msg, *shared_msg);
      plugin_publisher.publish(shared_msg);
      RCLCPP_WARN(rclcpp::get_logger("freform"), "Publishing to plugin publisher");
    }

    publish_raw(msg);
  }

  /*!
   * \brief Shutdown the advertisements associated with this Publisher.
   */
  IMAGE_TRANSPORT_PUBLIC
  void shutdown()
  {
    plugin_publisher.shutdown();
    raw_publisher.reset();
  }

  IMAGE_TRANSPORT_PUBLIC
  operator void *() const;

  IMAGE_TRANSPORT_PUBLIC
  bool operator<(const PublisherBase & rhs) const {return plugin_publisher < rhs.plugin_publisher;}

  IMAGE_TRANSPORT_PUBLIC
  bool operator!=(const PublisherBase & rhs) const
  {
    return plugin_publisher != rhs.plugin_publisher;
  }

  IMAGE_TRANSPORT_PUBLIC
  bool operator==(const PublisherBase & rhs) const
  {
    return plugin_publisher == rhs.plugin_publisher;
  }

private:
  template<typename T>
  void publish_raw(T && message) const
  {
    if (raw_publisher == nullptr) {
      // TODO(ros2) Switch to RCUTILS_ASSERT when ros2/rcutils#112 is merged
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "image_transport"), "Call to publish() on an invalid image_transport::Publisher");

      return;
    }

    if (raw_publisher->get_subscription_count() > 0) {
      raw_publisher->publish(std::forward<T>(message));
    }
  }

  bool HasPluginSubscribers() const
  {
    return plugin_publisher.getNumSubscribers() > 0;
  }

  std::string image_topic_;
  PluginPublisher plugin_publisher;
  typename rclcpp::Publisher<MessageT>::SharedPtr raw_publisher;
};

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__PUBLISHER_HPP_
