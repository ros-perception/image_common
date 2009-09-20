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
   * \brief Subscribe to a synchronized image & camera info topic pair.
   *
   * This version assumes the standard topic naming scheme, where the info topic is
   * named "camera_info" in the same namespace as the base image topic.
   */
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic,
                 uint32_t queue_size, const Callback& callback,
                 const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                 const TransportHints& transport_hints = TransportHints());

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair.
   *
   * Specify the info topic explicitly, for non-standard use cases.
   */
  void subscribe(ros::NodeHandle& nh,
                 const std::string& base_topic, const std::string& info_topic,
                 uint32_t queue_size, const Callback& callback,
                 const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                 const TransportHints& transport_hints = TransportHints());

  std::string getImageTopic() const;
  std::string getInfoTopic() const;

  /**
   * \brief Unsubscribe the callback associated with this CameraSubscriber.
   */
  void shutdown();
  
private:
  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

} //namespace image_transport

#endif
