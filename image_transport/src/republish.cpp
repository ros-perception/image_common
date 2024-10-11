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

#include "image_transport/republish.hpp"

#include <chrono>
#include <string>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>

#include "image_transport/image_transport.hpp"
#include "image_transport/publisher_plugin.hpp"

using namespace std::chrono_literals;

namespace image_transport
{

std::mutex pub_matched_mutex;

Republisher::Republisher(const rclcpp::NodeOptions & options)
: Node("image_republisher", options)
{
  // Initialize Republishercomponent after construction
  // shared_from_this can't be used in the constructor
  this->timer_ = create_wall_timer(
    1ms, [this]() {
      if (initialized_) {
        timer_->cancel();
      } else {
        this->initialize();
        initialized_ = true;
      }
    });
}

void Republisher::initialize()
{
  std::string in_topic = rclcpp::expand_topic_or_service_name(
    "in",
    this->get_name(), this->get_namespace());
  std::string out_topic = rclcpp::expand_topic_or_service_name(
    "out",
    this->get_name(), this->get_namespace());

  std::string in_transport = "raw";
  this->declare_parameter<std::string>("in_transport", in_transport);
  if (!this->get_parameter(
      "in_transport", in_transport))
  {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "The 'in_transport' parameter was not defined." << in_transport);
  } else {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "The 'in_transport' parameter is set to: " << in_transport);
  }

  std::string out_transport = "";
  this->declare_parameter<std::string>("out_transport", out_transport);
  if (!this->get_parameter(
      "out_transport", out_transport))
  {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "The parameter 'out_transport' was not defined." << out_transport);
  } else {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "The 'out_transport' parameter is set to: " << out_transport);
  }

  auto qos_override_options = rclcpp::QosOverridingOptions(
    {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
    });
  rclcpp::SubscriptionOptions sub_options;
  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = qos_override_options;
  sub_options.qos_overriding_options = qos_override_options;

  if (out_transport.empty()) {
    // Use all available transports for output

    // Use Publisher::publish as the subscriber callback
    typedef void (image_transport::Publisher::* PublishMemFn)(
      const sensor_msgs::msg::Image::ConstSharedPtr &) const;
    PublishMemFn pub_mem_fn = &image_transport::Publisher::publish;

    pub_options.event_callbacks.matched_callback =
      [this, in_topic, in_transport, pub_mem_fn, sub_options](rclcpp::MatchedInfo &)
      {
        std::scoped_lock<std::mutex> lock(pub_matched_mutex);
        if (this->pub.getNumSubscribers() == 0) {
          this->sub.shutdown();
        } else if (!this->sub) {
          this->sub = image_transport::create_subscription(
            this, in_topic,
            std::bind(pub_mem_fn, &this->pub, std::placeholders::_1),
            in_transport,
            rmw_qos_profile_default,
            sub_options);
        }
      };

    this->pub = image_transport::create_publisher(
      this, out_topic,
      rmw_qos_profile_default, pub_options);
  } else {
    // Use one specific transport for output
    // Load transport plugin
    typedef image_transport::PublisherPlugin Plugin;
    loader = std::make_shared<pluginlib::ClassLoader<Plugin>>(
      "image_transport",
      "image_transport::PublisherPlugin");
    std::string lookup_name = Plugin::getLookupName(out_transport);

    // Use PublisherPlugin::publishPtr as the subscriber callback
    typedef void (Plugin::* PublishMemFn)(const sensor_msgs::msg::Image::ConstSharedPtr &) const;
    PublishMemFn pub_mem_fn = &Plugin::publishPtr;

    this->instance = loader->createUniqueInstance(lookup_name);

    pub_options.event_callbacks.matched_callback =
      [this, in_topic, in_transport, pub_mem_fn, sub_options](rclcpp::MatchedInfo & matched_info)
      {
        if (matched_info.current_count == 0) {
          this->sub.shutdown();
        } else if (!this->sub) {
          this->sub = image_transport::create_subscription(
            this, in_topic,
            std::bind(
              pub_mem_fn,
              this->instance.get(), std::placeholders::_1), in_transport, rmw_qos_profile_default,
            sub_options);
        }
      };

    this->instance->advertise(this, out_topic, rmw_qos_profile_default, pub_options);
  }
}

}  // namespace image_transport

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_transport::Republisher)
