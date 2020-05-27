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

#ifndef IMAGE_TRANSPORT__SUBSCRIBER_FILTER_HPP_
#define IMAGE_TRANSPORT__SUBSCRIBER_FILTER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/simple_filter.h>

#include "image_transport/image_transport.hpp"
#include "image_transport/visibility_control.hpp"

namespace image_transport
{

/**
 * \brief Image subscription filter.
 *
 * This class wraps Subscriber as a "filter" compatible with the message_filters
 * package. It acts as a highest-level filter, simply passing messages from an image
 * transport subscription through to the filters which have connected to it.
 *
 * When this object is destroyed it will unsubscribe from the ROS subscription.
 *
 * \section connections CONNECTIONS
 *
 * SubscriberFilter has no input connection.
 *
 * The output connection for the SubscriberFilter object is the same signature as for roscpp
 * subscription callbacks, ie.
\verbatim
void callback(const std::shared_ptr<const sensor_msgs::msg::Image>&);
\endverbatim
 */

class SubscriberFilter : public message_filters::SimpleFilter<sensor_msgs::msg::Image>
{
public:
  /**
   * \brief Constructor
   *
   * See the ros::NodeHandle::subscribe() variants for more information on the parameters
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param base_topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport The transport hint to pass along
   */
  IMAGE_TRANSPORT_PUBLIC
  SubscriberFilter(
    rclcpp::Node * node, const std::string & base_topic,
    const std::string & transport)
  {
    subscribe(node, base_topic, transport);
  }

  /**
   * \brief Empty constructor, use subscribe() to subscribe to a topic
   */
  IMAGE_TRANSPORT_PUBLIC
  SubscriberFilter()
  {
  }

  IMAGE_TRANSPORT_PUBLIC
  ~SubscriberFilter()
  {
    unsubscribe();
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param base_topic The topic to subscribe to.
   */
  IMAGE_TRANSPORT_PUBLIC
  void subscribe(
    rclcpp::Node * node,
    const std::string & base_topic,
    const std::string & transport,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default)
  {
    unsubscribe();
    sub_ =
      image_transport::create_subscription(node, base_topic,
        std::bind(&SubscriberFilter::cb, this, std::placeholders::_1), transport, custom_qos);
  }

  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  IMAGE_TRANSPORT_PUBLIC
  void unsubscribe()
  {
    sub_.shutdown();
  }

  IMAGE_TRANSPORT_PUBLIC
  std::string getTopic() const
  {
    return sub_.getTopic();
  }

  /**
   * \brief Returns the number of publishers this subscriber is connected to.
   */
  IMAGE_TRANSPORT_PUBLIC
  size_t getNumPublishers() const
  {
    return sub_.getNumPublishers();
  }

  /**
   * \brief Returns the name of the transport being used.
   */
  IMAGE_TRANSPORT_PUBLIC
  std::string getTransport() const
  {
    return sub_.getTransport();
  }

  /**
   * \brief Returns the internal image_transport::Subscriber object.
   */
  IMAGE_TRANSPORT_PUBLIC
  const Subscriber & getSubscriber() const
  {
    return sub_;
  }

private:
  void cb(const sensor_msgs::msg::Image::ConstSharedPtr & m)
  {
    signalMessage(m);
  }

  Subscriber sub_;
};

}

#endif  // IMAGE_TRANSPORT__SUBSCRIBER_FILTER_HPP_
