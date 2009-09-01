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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <map>

namespace image_transport {

/**
 * \brief Publishes images efficiently across a bandwidth-limited network
 * connection.
 *
 * ImagePublisher will, on demand (i.e. if there are subscribers), publish
 * low-memory versions of the image message on separate topics. An ImagePublisher
 * constructed with base topic "camera/image" will advertise:
 *
 * - camera/image : The original image
 * - camera/image_thumbnail : The image scaled down to thumbnail size
 * - camera/image_compressed : A compressed (JPEG or PNG) version of the image
 */
class ImagePublisher
{
public:
  typedef std::map<std::string, std::string> TransportTopicMap;

  /*!
   * \brief Empty constructor, use advertise() to advertise a set of topics.
   */
  ImagePublisher();

  ImagePublisher(const ImagePublisher& rhs);
  
  ~ImagePublisher();

  // @todo: advertise overloads, support for subscription callbacks
  void advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                 bool latch = false);

  /*!
   * \brief Returns the number of subscribers that are currently connected to
   * this ImagePublisher.
   *
   * Returns the total number of subscribers to all advertised topics.
   */
  uint32_t getNumSubscribers() const;

  /*!
   * \brief Returns the base topic of this ImagePublisher.
   */
  std::string getTopic() const;

  TransportTopicMap& getTopicMap();
  const TransportTopicMap& getTopicMap() const;

  /*!
   * \brief Publish an image on the topics associated with this ImagePublisher.
   */
  void publish(const sensor_msgs::Image& message) const;

  /*!
   * \brief Publish an image on the topics associated with this ImagePublisher.
   */
  void publish(const sensor_msgs::ImageConstPtr& message) const;

  /*!
   * \brief Shutdown the advertisements associated with this ImagePublisher.
   */
  void shutdown();

  operator void*() const { return impl_ ? (void*)1 : (void*)0; }
  bool operator< (const ImagePublisher& rhs) const { return impl_ <  rhs.impl_; }
  bool operator!=(const ImagePublisher& rhs) const { return impl_ != rhs.impl_; }
  bool operator==(const ImagePublisher& rhs) const { return impl_ == rhs.impl_; }

private:
  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

} //namespace image_transport
