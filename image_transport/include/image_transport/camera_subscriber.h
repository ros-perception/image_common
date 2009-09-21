#ifndef IMAGE_TRANSPORT_CAMERA_SUBSCRIBER_H
#define IMAGE_TRANSPORT_CAMERA_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "image_transport/transport_hints.h"

namespace image_transport {

/**
 * \brief Subscribes to synchronized Image and CameraInfo topics.
 *
 * The image topic may be anything produced by Publisher. The callback
 * is of type:
\verbatim
void callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
\endverbatim
 */
class CameraSubscriber
{
public:
  typedef boost::function<void(const sensor_msgs::ImageConstPtr&,
                               const sensor_msgs::CameraInfoConstPtr&)> Callback;
  
  CameraSubscriber();

  CameraSubscriber(const CameraSubscriber& rhs);

  ~CameraSubscriber();

  /**
   * \brief Get the base topic (on which the raw image is published).
   */
  std::string getTopic() const;

  /**
   * \brief Get the camera info topic.
   */
  std::string getInfoTopic() const;

  /**
   * \brief Unsubscribe the callback associated with this CameraSubscriber.
   */
  void shutdown();

  operator void*() const;
  bool operator< (const CameraSubscriber& rhs) const { return impl_ <  rhs.impl_; }
  bool operator!=(const CameraSubscriber& rhs) const { return impl_ != rhs.impl_; }
  bool operator==(const CameraSubscriber& rhs) const { return impl_ == rhs.impl_; }
  
private:
  CameraSubscriber(ros::NodeHandle& nh, const std::string& base_topic,
                   uint32_t queue_size, const Callback& callback,
                   const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                   const TransportHints& transport_hints = TransportHints());
  
  struct Impl;
  boost::shared_ptr<Impl> impl_;

  friend class ImageTransport;
};

} //namespace image_transport

#endif
