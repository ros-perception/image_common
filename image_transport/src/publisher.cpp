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

#include "image_transport/publisher.hpp"

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

#include "pluginlib/class_loader.hpp"

#include "image_transport/camera_common.hpp"
#include "image_transport/publisher_plugin.hpp"

namespace image_transport
{

struct Publisher::Impl
{
  explicit Impl(rclcpp::Node * node)
  : logger_(node->get_logger()),
    unadvertised_(false)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  size_t getNumSubscribers() const
  {
    size_t count = 0;
    for (const auto & pub : publishers_) {
      count += pub->getNumSubscribers();
    }
    return count;
  }

  std::string getTopic() const
  {
    return base_topic_;
  }

  bool isValid() const
  {
    return !unadvertised_;
  }

  void shutdown()
  {
    if (!unadvertised_) {
      unadvertised_ = true;
      for (auto & pub : publishers_) {
        pub->shutdown();
      }
      publishers_.clear();
    }
  }

  rclcpp::Logger logger_;
  std::string base_topic_;
  PubLoaderPtr loader_;
  std::vector<std::shared_ptr<PublisherPlugin>> publishers_;
  bool unadvertised_;
};

Publisher::Publisher(
  rclcpp::Node * node, const std::string & base_topic,
  PubLoaderPtr loader, rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
: impl_(std::make_shared<Impl>(node))
{
  // Resolve the name explicitly because otherwise the compressed topics don't remap
  // properly (#3652).
  std::string image_topic = rclcpp::expand_topic_or_service_name(
    base_topic,
    node->get_name(), node->get_namespace());
  impl_->base_topic_ = image_topic;
  impl_->loader_ = loader;

  auto ns_len = node->get_effective_namespace().length();
  std::string param_base_name = image_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  if (param_base_name.front() == '.') {
    param_base_name = param_base_name.substr(1);
  }
  std::vector<std::string> allowlist_vec;
  std::set<std::string> allowlist;
  std::vector<std::string> all_transport_names;
  for (const auto & lookup_name : loader->getDeclaredClasses()) {
    all_transport_names.emplace_back(erase_last_copy(lookup_name, "_pub"));
  }
  try {
    allowlist_vec = node->declare_parameter<std::vector<std::string>>(
      param_base_name + ".enable_pub_plugins", all_transport_names);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG_STREAM(
      node->get_logger(), param_base_name << ".enable_pub_plugins" << " was previously declared"
    );
    allowlist_vec =
      node->get_parameter(
      param_base_name +
      ".enable_pub_plugins").get_value<std::vector<std::string>>();
  }
  for (size_t i = 0; i < allowlist_vec.size(); ++i) {
    allowlist.insert(allowlist_vec[i]);
  }

  for (const auto & transport_name : allowlist) {
    const auto & lookup_name = transport_name + "_pub";
    try {
      auto pub = loader->createUniqueInstance(lookup_name);
      pub->advertise(node, image_topic, custom_qos, options);
      impl_->publishers_.push_back(std::move(pub));
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(
        impl_->logger_, "Failed to load plugin %s, error string: %s\n",
        lookup_name.c_str(), e.what());
    }
  }

  if (impl_->publishers_.empty()) {
    throw Exception(
            "No plugins found! Does `rospack plugins --attrib=plugin "
            "image_transport` find any packages?");
  }
}

size_t Publisher::getNumSubscribers() const
{
  if (impl_ && impl_->isValid()) {return impl_->getNumSubscribers();}
  return 0;
}

std::string Publisher::getTopic() const
{
  if (impl_) {return impl_->getTopic();}
  return std::string();
}

void Publisher::publish(const sensor_msgs::msg::Image & message) const
{
  if (!impl_ || !impl_->isValid()) {
    // TODO(ros2) Switch to RCUTILS_ASSERT when ros2/rcutils#112 is merged
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(logger, "Call to publish() on an invalid image_transport::Publisher");
    return;
  }

  for (const auto & pub : impl_->publishers_) {
    if (pub->getNumSubscribers() > 0) {
      pub->publish(message);
    }
  }
}

void Publisher::publish(const sensor_msgs::msg::Image::ConstSharedPtr & message) const
{
  if (!impl_ || !impl_->isValid()) {
    // TODO(ros2) Switch to RCUTILS_ASSERT when ros2/rcutils#112 is merged
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(logger, "Call to publish() on an invalid image_transport::Publisher");
    return;
  }

  for (const auto & pub : impl_->publishers_) {
    if (pub->getNumSubscribers() > 0) {
      pub->publishPtr(message);
    }
  }
}

void Publisher::publish(sensor_msgs::msg::Image::UniquePtr message) const
{
  if (!impl_ || !impl_->isValid()) {
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("image_transport");
    RCLCPP_FATAL(logger, "Call to publish() on an invalid image_transport::Publisher");
    return;
  }

  std::vector<std::shared_ptr<PublisherPlugin>> pubs_take_reference;
  std::optional<std::shared_ptr<PublisherPlugin>> pub_takes_ownership{std::nullopt};

  for (const auto & pub : impl_->publishers_) {
    if (pub->getNumSubscribers() > 0) {
      if (pub->supportsUniquePtrPub() && !pub_takes_ownership.has_value()) {
        pub_takes_ownership = pub;
      } else {
        pubs_take_reference.push_back(pub);
      }
    }
  }

  for (const auto & pub : pubs_take_reference) {
    pub->publish(*message);
  }

  if (pub_takes_ownership.has_value()) {
    pub_takes_ownership.value()->publishUniquePtr(std::move(message));
  }
}

void Publisher::shutdown()
{
  if (impl_) {
    impl_->shutdown();
    impl_.reset();
  }
}

Publisher::operator void *() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void *>(1) : reinterpret_cast<void *>(0);
}

}  // namespace image_transport
