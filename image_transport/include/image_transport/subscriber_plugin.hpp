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

#ifndef IMAGE_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_
#define IMAGE_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_

#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_transport/visibility_control.hpp"

namespace image_transport
{

/**
 * \brief Base class for plugins to Subscriber.
 */
class IMAGE_TRANSPORT_PUBLIC SubscriberPlugin
{
public:
  SubscriberPlugin() = default;
  SubscriberPlugin(const SubscriberPlugin &) = delete;
  SubscriberPlugin & operator=(const SubscriberPlugin &) = delete;

  virtual ~SubscriberPlugin() {}

  typedef std::function<void (const sensor_msgs::msg::Image::ConstSharedPtr &)> Callback;

  /**
   * \brief Get a string identifier for the transport provided by
   * this plugin.
   */
  virtual std::string getTransportName() const = 0;

  /**
   * \brief Subscribe to an image topic, version for arbitrary std::function object.
   */
  void subscribe(
    rclcpp::Node * node, const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribeImpl(node, base_topic, callback, custom_qos);
  }

  /**
   * \brief Subscribe to an image topic, version for bare function.
   */
  void subscribe(
    rclcpp::Node * node, const std::string & base_topic,
    void (*fp)(const sensor_msgs::msg::Image::ConstSharedPtr &),
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe(node, base_topic,
             std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &)>(fp),
             custom_qos);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with bare pointer.
   */
  template<class T>
  void subscribe(
    rclcpp::Node * node, const std::string & base_topic,
    void (T::* fp)(const sensor_msgs::msg::Image::ConstSharedPtr &), T * obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe(node, base_topic,
             std::bind(fp, obj, std::placeholders::_1), custom_qos);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with shared_ptr.
   */
  template<class T>
  void subscribe(
    rclcpp::Node * node, const std::string & base_topic,
    void (T::* fp)(const sensor_msgs::msg::Image::ConstSharedPtr &),
    std::shared_ptr<T> & obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    return subscribe(node, base_topic,
             std::bind(fp, obj, std::placeholders::_1), custom_qos);
  }

  /**
   * \brief Get the transport-specific communication topic.
   */
  virtual std::string getTopic() const = 0;

  /**
   * \brief Returns the number of publishers this subscriber is connected to.
   */
  virtual size_t getNumPublishers() const = 0;

  /**
   * \brief Unsubscribe the callback associated with this SubscriberPlugin.
   */
  virtual void shutdown() = 0;

  /**
   * \brief Return the lookup name of the SubscriberPlugin associated with a specific
   * transport identifier.
   */
  static std::string getLookupName(const std::string & transport_type)
  {
    return "image_transport/" + transport_type + "_sub";
  }

protected:
  /**
   * \brief Subscribe to an image transport topic. Must be implemented by the subclass.
   */
  virtual void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default) = 0;
};

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_
