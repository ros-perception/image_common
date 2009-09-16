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

#ifndef IMAGE_TRANSPORT_IMAGE_SUBSCRIBER_FILTER_H
#define IMAGE_TRANSPORT_IMAGE_SUBSCRIBER_FILTER_H

#include <ros/ros.h>
#include <message_filters/simple_filter.h>

#include "image_transport/image_subscriber.h"

namespace image_transport {

/**
 * \brief Image subscription filter.
 *
 * This class wraps ImageSubscriber as a "filter" compatible with the message_filters
 * package. It acts as a highest-level filter, simply passing messages from an image
 * transport subscription through to the filters which have connected to it.
 *
 * When this object is destroyed it will unsubscribe from the ROS subscription.
 *
 * \section connections CONNECTIONS
 *
 * ImageSubscriberFilter has no input connection.
 *
 * The output connection for the ImageSubscriberFilter object is the same signature as for roscpp
 * subscription callbacks, ie.
\verbatim
void callback(const boost::shared_ptr<const sensor_msgs::Image>&);
\endverbatim
 */
class ImageSubscriberFilter : public message_filters::SimpleFilter<sensor_msgs::Image>
{
public:
  /**
   * \brief Constructor
   *
   * See the ros::NodeHandle::subscribe() variants for more information on the parameters
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport_hints The transport hints to pass along
   * \param callback_queue The callback queue to pass along
   */
  ImageSubscriberFilter(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                        const ros::TransportHints& transport_hints = ros::TransportHints(),
                        ros::CallbackQueueInterface* callback_queue = 0)
  {
    subscribe(nh, topic, queue_size, transport_hints, callback_queue);
  }

  /**
   * \brief Empty constructor, use subscribe() to subscribe to a topic
   */
  ImageSubscriberFilter()
  {
  }

  ~ImageSubscriberFilter()
  {
    unsubscribe();
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport_hints The transport hints to pass along
   * \param callback_queue The callback queue to pass along
   */
  void subscribe(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                 const ros::TransportHints& transport_hints = ros::TransportHints(),
                 ros::CallbackQueueInterface* callback_queue = 0)
  {
    unsubscribe();

    if (!topic.empty())
    {
      //! @todo currently ignoring callback_queue
      sub_.subscribe(nh, topic, queue_size, boost::bind(&ImageSubscriberFilter::cb, this, _1),
                     ros::VoidPtr(), transport_hints);
    }
  }

  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  void unsubscribe()
  {
    sub_.shutdown();
  }

private:

  void cb(const sensor_msgs::ImageConstPtr& m)
  {
    signalMessage(m);
  }

  ImageSubscriber sub_;
};

}

#endif
