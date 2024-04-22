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

#include "image_transport/subscriber.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "pluginlib/class_loader.hpp"

#include "image_transport/subscriber_plugin.hpp"

namespace image_transport
{

template<class NodeType>
struct Subscriber<NodeType>::Impl
{
  Impl(NodeType * node, SubLoaderPtr<NodeType> loader)
  : node_(node),
    logger_(node->get_logger()),
    loader_(loader),
    unsubscribed_(false)
  {
  }

  explicit Impl(std::shared_ptr<NodeType> node, SubLoaderPtr<NodeType> loader)
  : node_(node),
    logger_(node->get_logger()),
    loader_(loader),
    unsubscribed_(false)
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
      if (subscriber_) {
        subscriber_->shutdown();
      }
    }
  }

  std::shared_ptr<NodeType> node_;
  rclcpp::Logger logger_;
  std::string lookup_name_;
  SubLoaderPtr<NodeType> loader_;
  std::shared_ptr<SubscriberPlugin<NodeType>> subscriber_;
  bool unsubscribed_;
  // double constructed_;
};

template<class NodeType>
Subscriber<NodeType>::Subscriber(
  NodeType * node,
  const std::string & base_topic,
  const Callback & callback,
  SubLoaderPtr<NodeType> loader,
  const std::string & transport,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
: impl_(std::make_shared<Impl>(node, loader))
{
  initialise(base_topic, callback, transport, custom_qos, options);
}

template<class NodeType>
Subscriber<NodeType>::Subscriber(
  std::shared_ptr<NodeType> node,
  const std::string & base_topic,
  const Callback & callback,
  SubLoaderPtr<NodeType> loader,
  const std::string & transport,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
: impl_(std::make_shared<Impl>(node, loader))
{
  initialise(base_topic, callback, transport, custom_qos, options);
}

template<class NodeType>
void Subscriber<NodeType>::initialise(
  const std::string & base_topic,
  const Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  if (!impl_) {
    throw std::runtime_error("impl is not constructed!");
  }
  // Load the plugin for the chosen transport.
  impl_->lookup_name_ = SubscriberPlugin<NodeType>::getLookupName(transport);
  try {
    impl_->subscriber_ = impl_->loader_->createSharedInstance(impl_->lookup_name_);
  } catch (pluginlib::PluginlibException & e) {
    throw TransportLoadException(impl_->lookup_name_, e.what());
  }

  // Try to catch if user passed in a transport-specific topic as base_topic.
  // TODO(ros2) use rclcpp to clean
  // std::string clean_topic = ros::names::clean(base_topic);
  std::string clean_topic = base_topic;

  size_t found = clean_topic.rfind('/');
  if (found != std::string::npos) {
    std::string transport = clean_topic.substr(found + 1);
    std::string plugin_name = SubscriberPlugin<NodeType>::getLookupName(transport);
    std::vector<std::string> plugins = impl_->loader_->getDeclaredClasses();
    if (std::find(plugins.begin(), plugins.end(), plugin_name) != plugins.end()) {
      std::string real_base_topic = clean_topic.substr(0, found);

      RCLCPP_WARN(
        impl_->logger_,
        "[image_transport] It looks like you are trying to subscribe directly to a "
        "transport-specific image topic '%s', in which case you will likely get a connection "
        "error. Try subscribing to the base topic '%s' instead with parameter ~image_transport "
        "set to '%s' (on the command line, _image_transport:=%s). "
        "See http://ros.org/wiki/image_transport for details.",
        clean_topic.c_str(), real_base_topic.c_str(), transport.c_str(), transport.c_str());
    }
  }

  // Tell plugin to subscribe.
  RCLCPP_DEBUG(impl_->logger_, "Subscribing to: %s\n", base_topic.c_str());
  impl_->subscriber_->subscribe(impl_->node_, base_topic, callback, custom_qos, options);
}

template<class NodeType>
std::string Subscriber<NodeType>::getTopic() const
{
  if (impl_) {return impl_->subscriber_->getTopic();}
  return std::string();
}

template<class NodeType>
size_t Subscriber<NodeType>::getNumPublishers() const
{
  if (impl_) {return impl_->subscriber_->getNumPublishers();}
  return 0;
}

template<class NodeType>
std::string Subscriber<NodeType>::getTransport() const
{
  if (impl_) {return impl_->subscriber_->getTransportName();}
  return std::string();
}

template<class NodeType>
void Subscriber<NodeType>::shutdown()
{
  if (impl_) {impl_->shutdown();}
}

template<class NodeType>
Subscriber<NodeType>::operator void *() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void *>(1) : reinterpret_cast<void *>(0);
}

}  // namespace image_transport

template class image_transport::Subscriber<rclcpp::Node>;
template class image_transport::Subscriber<rclcpp_lifecycle::LifecycleNode>;
