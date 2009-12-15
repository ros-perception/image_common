#ifndef IMAGE_TRANSPORT_CAMERA_SUBSCRIBER_H
#define IMAGE_TRANSPORT_CAMERA_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "image_transport/transport_hints.h"

namespace image_transport {

class ImageTransport;

/**
 * \brief Manages a subscription callback on synchronized Image and CameraInfo topics.
 *
 * CameraSubscriber is the client-side counterpart to CameraPublisher, and assumes the
 * same topic naming convention. The callback is of type:
\verbatim
void callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
\endverbatim
 *
 * A CameraSubscriber should always be created through a call to
 * ImageTransport::subscribeCamera(), or copied from one that was.
 * Once all copies of a specific CameraSubscriber go out of scope, the subscription callback
 * associated with that handle will stop being called. Once all CameraSubscriber for a given
 * topic go out of scope the topic will be unsubscribed.
 */
class CameraSubscriber
{
public:
  typedef boost::function<void(const sensor_msgs::ImageConstPtr&,
                               const sensor_msgs::CameraInfoConstPtr&)> Callback;
  
  CameraSubscriber() {}

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
  CameraSubscriber(ImageTransport& image_it, ros::NodeHandle& info_nh,
                   const std::string& base_topic, uint32_t queue_size,
                   const Callback& callback,
                   const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                   const TransportHints& transport_hints = TransportHints());
  
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;
  
  ImplPtr impl_;

  friend class ImageTransport;
};

} //namespace image_transport

#endif
