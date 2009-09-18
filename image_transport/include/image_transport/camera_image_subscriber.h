#ifndef IMAGE_TRANSPORT_CAMERA_SUBSCRIBER_H
#define IMAGE_TRANSPORT_CAMERA_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

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

  ~CameraSubscriber();

  /**
   * \brief Subscribe to an image topic, inferring the camera info topic name.
   *
   * The info topic is assumed to be named "camera_info" in the same namespace
   * as the image topic.
   */
  void subscribe(ros::NodeHandle& nh, const std::string& image_topic,
                 uint32_t queue_size, const Callback& callback,
                 const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                 const ros::TransportHints& transport_hints = ros::TransportHints());

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair.
   */
  void subscribe(ros::NodeHandle& nh,
                 const std::string& image_topic, const std::string& info_topic,
                 uint32_t queue_size, const Callback& callback,
                 const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                 const ros::TransportHints& transport_hints = ros::TransportHints());

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
