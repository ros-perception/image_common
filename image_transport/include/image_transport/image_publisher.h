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

#ifndef IMAGE_TRANSPORT_PUBLISHER_H
#define IMAGE_TRANSPORT_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <map>

namespace image_transport {

/**
 * \brief Manages advertisements of multiple transport options on an Image topic.
 *
 * Publisher is a (nearly) drop-in replacement for ros::Publisher when publishing
 * Image topics. In a minimally built environment, they behave the same; however,
 * Publisher is extensible via plugins to publish alternative representations of
 * the image on related subtopics. This is especially useful for limiting bandwidth and
 * latency over a network connection, when you might (for example) use the theora plugin
 * to transport the images as streamed video. All topics are published only on demand
 * (i.e. if there are subscribers).
 * 
 * Once all copies of a specific Publisher go out of scope, any subscriber callbacks
 * associated with that handle will stop being called. Once all Publisher for a
 * given base topic go out of scope the topic (and all subtopics) will be unadvertised.
 */
class Publisher
{
public:
  typedef std::map<std::string, std::string> TransportTopicMap;

  /*!
   * \brief Empty constructor, use advertise() to advertise a set of topics.
   */
  Publisher();

  Publisher(const Publisher& rhs);
  
  ~Publisher();

  /*!
   * \brief Advertise a topic with subcriber status callbacks.
   */
  void advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                 const ros::SubscriberStatusCallback& connect_cb,
                 const ros::SubscriberStatusCallback& disconnect_cb = ros::SubscriberStatusCallback(),
                 const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);

  /*!
   * \brief Advertise a topic, simple version.
   */
  void advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                 bool latch = false);

  /*!
   * \brief Returns the number of subscribers that are currently connected to
   * this Publisher.
   *
   * Returns the total number of subscribers to all advertised topics.
   */
  uint32_t getNumSubscribers() const;

  /*!
   * \brief Returns the base topic of this Publisher.
   */
  std::string getTopic() const;

  /*!
   * \brief Get the mappings from transport type to topic name, mutable version.
   *
   * By default, the topic map will be populated with every available transport
   * plugin, each mapped to its default extension of the base topic name.
   */
  TransportTopicMap& getTopicMap();

  /*!
   * \brief Get the mappings from transport type to topic name, const version.
   */
  const TransportTopicMap& getTopicMap() const;

  /*!
   * \brief Publish an image on the topics associated with this Publisher.
   */
  void publish(const sensor_msgs::Image& message) const;

  /*!
   * \brief Publish an image on the topics associated with this Publisher.
   */
  void publish(const sensor_msgs::ImageConstPtr& message) const;

  /*!
   * \brief Shutdown the advertisements associated with this Publisher.
   */
  void shutdown();

  operator void*() const { return impl_ ? (void*)1 : (void*)0; }
  bool operator< (const Publisher& rhs) const { return impl_ <  rhs.impl_; }
  bool operator!=(const Publisher& rhs) const { return impl_ != rhs.impl_; }
  bool operator==(const Publisher& rhs) const { return impl_ == rhs.impl_; }

private:
  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

} //namespace image_transport

#endif
