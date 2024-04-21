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

#include "image_transport/image_transport.hpp"

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "image_transport/camera_common.hpp"
#include "image_transport/loader_fwds.hpp"
#include "image_transport/publisher_plugin.hpp"
#include "image_transport/subscriber_plugin.hpp"

namespace image_transport
{

template class ImageTransport<rclcpp::Node>;
template class ImageTransport<rclcpp_lifecycle::LifecycleNode>;


struct Impl
{
  PubLoaderPtr<rclcpp::Node> pub_loader_;
  SubLoaderPtr<rclcpp::Node> sub_loader_;

  Impl()
  : pub_loader_(std::make_shared<PubLoader<rclcpp::Node>>("image_transport", "image_transport::PublisherPlugin<rclcpp::Node>")),
    sub_loader_(std::make_shared<SubLoader<rclcpp::Node>>("image_transport", "image_transport::SubscriberPlugin<rclcpp::Node>"))
  {
  }
};

struct ImplLifecycle
{
  PubLoaderPtr<rclcpp_lifecycle::LifecycleNode> pub_loader_;
  SubLoaderPtr<rclcpp_lifecycle::LifecycleNode> sub_loader_;

  ImplLifecycle()
  : pub_loader_(std::make_shared<PubLoader<rclcpp_lifecycle::LifecycleNode>>("image_transport", "image_transport::PublisherPlugin<rclcpp_lifecycle::LifecycleNode>")),
    sub_loader_(std::make_shared<SubLoader<rclcpp_lifecycle::LifecycleNode>>("image_transport", "image_transport::SubscriberPlugin<rclcpp_lifecycle::LifecycleNode>"))
  {
  }
};

static std::unique_ptr<Impl> kImpl = std::make_unique<Impl>();
static std::unique_ptr<ImplLifecycle> kImpl_lifecycle = std::make_unique<ImplLifecycle>();

template<class NodeType>
Publisher<NodeType> create_publisher(
  NodeType * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  if constexpr (std::is_same_v<NodeType, rclcpp::Node>)
  {
    return Publisher(node, base_topic, kImpl->pub_loader_, custom_qos, options);
  }
  if constexpr (std::is_same_v<NodeType, rclcpp_lifecycle::LifecycleNode>)
  {
    return Publisher(node, base_topic, kImpl_lifecycle->pub_loader_, custom_qos, options);
  }
}

template<class NodeType>
Publisher<NodeType> create_publisher(
  std::shared_ptr<NodeType> node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  if constexpr (std::is_same_v<NodeType, rclcpp::Node>)
  {
    return Publisher(node, base_topic, kImpl->pub_loader_, custom_qos, options);
  }
  if constexpr (std::is_same_v<NodeType, rclcpp_lifecycle::LifecycleNode>)
  {
    return Publisher(node, base_topic, kImpl_lifecycle->pub_loader_, custom_qos, options);
  }
}

template<class NodeType>
Subscriber<NodeType> create_subscription(
  NodeType * node,
  const std::string & base_topic,
  const typename Subscriber<NodeType>::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  if constexpr (std::is_same_v<NodeType, rclcpp::Node>)
  {
    return Subscriber(node, base_topic, callback, kImpl->sub_loader_, transport, custom_qos, options);
  }
  if constexpr (std::is_same_v<NodeType, rclcpp_lifecycle::LifecycleNode>)
  {
    return Subscriber(node, base_topic, callback, kImpl_lifecycle->sub_loader_, transport + "_lifecycle", custom_qos, options);
  }
}

template<class NodeType>
Subscriber<NodeType> create_subscription(
  std::shared_ptr<NodeType> node,
  const std::string & base_topic,
  const typename Subscriber<NodeType>::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  if constexpr (std::is_same_v<NodeType, rclcpp::Node>)
  {
    return Subscriber(node, base_topic, callback, kImpl->sub_loader_, transport, custom_qos, options);
  }
  if constexpr (std::is_same_v<NodeType, rclcpp_lifecycle::LifecycleNode>)
  {
    return Subscriber(node, base_topic, callback, kImpl_lifecycle->sub_loader_, transport + "_lifecycle", custom_qos, options);
  }
}

template<class NodeType>
CameraPublisher<NodeType> create_camera_publisher(
  NodeType * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions pub_options)
{
  return CameraPublisher(node, base_topic, custom_qos, pub_options);
}

template<class NodeType>
CameraPublisher<NodeType> create_camera_publisher(
  std::shared_ptr<NodeType> node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions pub_options)
{
  return CameraPublisher(node, base_topic, custom_qos, pub_options);
}

template<class NodeType>
CameraSubscriber<NodeType> create_camera_subscription(
  NodeType * node,
  const std::string & base_topic,
  const typename CameraSubscriber<NodeType>::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos)
{
  return CameraSubscriber(node, base_topic, callback, transport, custom_qos);
}

template<class NodeType>
CameraSubscriber<NodeType> create_camera_subscription(
  std::shared_ptr<NodeType> node,
  const std::string & base_topic,
  const typename CameraSubscriber<NodeType>::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos)
{
  return CameraSubscriber(node, base_topic, callback, transport, custom_qos);
}

template<class NodeType>
std::vector<std::string> getDeclaredTransports()
{
  std::vector<std::string> transports;
  if constexpr (std::is_same_v<NodeType, rclcpp::Node>)
  {
    transports = kImpl->sub_loader_->getDeclaredClasses();
  }
  if constexpr (std::is_same_v<NodeType, rclcpp_lifecycle::LifecycleNode>)
  {
    transports = kImpl_lifecycle->sub_loader_->getDeclaredClasses();
  }
  // Remove the "_sub" at the end of each class name.
  for (std::string & transport : transports) {
    transport = erase_last_copy(transport, "_sub");
  }
  return transports;
}

template<class NodeType>
std::vector<std::string> getLoadableTransports()
{
  std::vector<std::string> loadableTransports;
  std::vector<std::string> transports;
  if constexpr (std::is_same_v<NodeType, rclcpp::Node>)
  {
    transports = kImpl->sub_loader_->getDeclaredClasses();
  }
  if constexpr (std::is_same_v<NodeType, rclcpp_lifecycle::LifecycleNode>)
  {
    transports = kImpl_lifecycle->sub_loader_->getDeclaredClasses();
  }
  for (const std::string & transportPlugin : transports ) {
    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try {
      std::shared_ptr<image_transport::SubscriberPlugin<NodeType>> sub;
      if constexpr (std::is_same_v<NodeType, rclcpp::Node>)
      {
        sub = kImpl->sub_loader_->createUniqueInstance(transportPlugin);
      }
      if constexpr (std::is_same_v<NodeType, rclcpp_lifecycle::LifecycleNode>)
      {
        sub = kImpl_lifecycle->sub_loader_->createUniqueInstance(transportPlugin);
      }
      // Remove the "_sub" at the end of each class name.
      loadableTransports.push_back(erase_last_copy(transportPlugin, "_sub"));
    } catch (const pluginlib::LibraryLoadException & e) {
      (void) e;
    } catch (const pluginlib::CreateClassException & e) {
      (void) e;
    }
  }

  return loadableTransports;
}

template<class NodeType>
struct ImageTransport<NodeType>::Impl
{
  explicit Impl(NodeType * node)
  : node_(node)
  {
  }

  explicit Impl(std::shared_ptr<NodeType> node)
  : node_(node)
  {
  }

  std::shared_ptr<NodeType> node_;
};

template<class NodeType>
ImageTransport<NodeType>::ImageTransport(NodeType * node)
: impl_(std::make_unique<ImageTransport<NodeType>::Impl>(node))
{
}

template<class NodeType>
ImageTransport<NodeType>::ImageTransport(std::shared_ptr<NodeType> node)
: impl_(std::make_unique<ImageTransport<NodeType>::Impl>(node))
{
}

template<class NodeType>
ImageTransport<NodeType>::~ImageTransport() = default;

template<class NodeType>
Publisher<NodeType> ImageTransport<NodeType>::advertise(const std::string & base_topic, uint32_t queue_size, bool latch)
{
  // TODO(ros2) implement when resolved: https://github.com/ros2/ros2/issues/464
  (void) latch;
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  custom_qos.depth = queue_size;
  return create_publisher(impl_->node_, base_topic, custom_qos);
}

template<class NodeType>
Publisher<NodeType> ImageTransport<NodeType>::advertise(
  const std::string & base_topic, rmw_qos_profile_t custom_qos,
  bool latch)
{
  // TODO(ros2) implement when resolved: https://github.com/ros2/ros2/issues/464
  (void) latch;
  return create_publisher(impl_->node_, base_topic, custom_qos);
}

template<class NodeType>
Subscriber<NodeType> ImageTransport<NodeType>::subscribe(
  const std::string & base_topic, rmw_qos_profile_t custom_qos,
  const typename Subscriber<NodeType>::Callback & callback,
  const VoidPtr & tracked_object,
  const TransportHints * transport_hints,
  const rclcpp::SubscriptionOptions options)
{
  (void) tracked_object;
  return create_subscription(
    impl_->node_, base_topic, callback,
    getTransportOrDefault(transport_hints), custom_qos,
    options);
}

template<class NodeType>
Subscriber<NodeType> ImageTransport<NodeType>::subscribe(
  const std::string & base_topic, uint32_t queue_size,
  const typename Subscriber<NodeType>::Callback & callback,
  const VoidPtr & tracked_object,
  const TransportHints * transport_hints,
  const rclcpp::SubscriptionOptions options)
{
  (void) tracked_object;
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  custom_qos.depth = queue_size;
  return create_subscription(
    impl_->node_, base_topic, callback,
    getTransportOrDefault(transport_hints), custom_qos,
    options);
}

template<class NodeType>
CameraPublisher<NodeType> ImageTransport<NodeType>::advertiseCamera(
  const std::string & base_topic, uint32_t queue_size,
  bool latch)
{
  // TODO(ros2) implement when resolved: https://github.com/ros2/ros2/issues/464
  (void) latch;
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  custom_qos.depth = queue_size;
  return create_camera_publisher(impl_->node_, base_topic, custom_qos);
}

template<class NodeType>
CameraSubscriber<NodeType> ImageTransport<NodeType>::subscribeCamera(
  const std::string & base_topic, uint32_t queue_size,
  const typename CameraSubscriber<NodeType>::Callback & callback,
  const VoidPtr & tracked_object,
  const TransportHints * transport_hints)
{
  (void) tracked_object;
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  custom_qos.depth = queue_size;
  return create_camera_subscription(
    impl_->node_, base_topic, callback,
    getTransportOrDefault(transport_hints), custom_qos);
}

template<class NodeType>
std::vector<std::string> ImageTransport<NodeType>::getDeclaredTransports() const
{
  return image_transport::getDeclaredTransports();
}

template<class NodeType>
std::vector<std::string> ImageTransport<NodeType>::getLoadableTransports() const
{
  return image_transport::getLoadableTransports();
}

template<class NodeType>
std::string ImageTransport<NodeType>::getTransportOrDefault(const TransportHints * transport_hints)
{
  std::string ret;
  if (nullptr == transport_hints) {
    TransportHints th(impl_->node_);
    ret = th.getTransport();
  } else {
    ret = transport_hints->getTransport();
  }
  return ret;
}

}  // namespace image_transport
