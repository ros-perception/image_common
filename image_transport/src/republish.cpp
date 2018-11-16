/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.h"
#include "image_transport/publisher_plugin.h"
#include <pluginlib/class_loader.hpp>

int main(int argc, char ** argv)
{
  auto vargv = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (vargv.size() < 2) {
    printf("Usage: %s in_transport in:=<in_base_topic> [out_transport] out:=<out_base_topic>\n",
      argv[0]);
    return 0;
  }

  auto node = rclcpp::Node::make_shared("image_republisher");

  std::string in_topic = rclcpp::expand_topic_or_service_name("in",
      node->get_name(), node->get_namespace());
  std::string out_topic = rclcpp::expand_topic_or_service_name("out",
      node->get_name(), node->get_namespace());

  std::string in_transport = vargv[1];

  if (vargv.size() < 3) {
    // Use all available transports for output
    auto pub = image_transport::create_publisher(node.get(), out_topic);

    // Use Publisher::publish as the subscriber callback
    typedef void (image_transport::Publisher::* PublishMemFn)(const sensor_msgs::msg::Image::
      ConstSharedPtr &) const;
    PublishMemFn pub_mem_fn = &image_transport::Publisher::publish;

    auto sub =
      image_transport::create_subscription(node.get(), in_topic,
        std::bind(pub_mem_fn, &pub, std::placeholders::_1), in_transport);
    rclcpp::spin(node);
  } else {
    // Use one specific transport for output
    std::string out_transport = vargv[2];

    // Load transport plugin
    typedef image_transport::PublisherPlugin Plugin;
    pluginlib::ClassLoader<Plugin> loader("image_transport", "image_transport::PublisherPlugin");
    std::string lookup_name = Plugin::getLookupName(out_transport);

    auto instance = loader.createUniqueInstance(lookup_name);
    std::shared_ptr<Plugin> pub = std::move(instance);
    pub->advertise(node.get(), out_topic);

    // Use PublisherPlugin::publish as the subscriber callback
    typedef void (Plugin::* PublishMemFn)(const sensor_msgs::msg::Image::ConstSharedPtr &) const;
    PublishMemFn pub_mem_fn = &Plugin::publish;
    auto sub =
      image_transport::create_subscription(node.get(), in_topic,
        std::bind(pub_mem_fn, pub.get(), std::placeholders::_1), in_transport);
    rclcpp::spin(node);
  }

  return 0;
}
