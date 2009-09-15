#ifndef IMAGE_TRANSPORT_CAMERA_IMAGE_SUBSCRIBER_H
#define IMAGE_TRANSPORT_CAMERA_IMAGE_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace image_transport
{
class CameraImageSubscriber
{
public:
  typedef boost::function<void(const sensor_msgs::ImageConstPtr&,
                               const sensor_msgs::CameraInfoConstPtr&)> Callback;
  
  CameraImageSubscriber();

  ~CameraImageSubscriber();

  void subscribe(ros::NodeHandle& nh, const std::string& image_topic,
                 uint32_t queue_size, const Callback& callback,
                 const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                 const ros::TransportHints& transport_hints = ros::TransportHints());
  
  void subscribe(ros::NodeHandle& nh,
                 const std::string& image_topic, const std::string& info_topic,
                 uint32_t queue_size, const Callback& callback,
                 const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                 const ros::TransportHints& transport_hints = ros::TransportHints());

  std::string getImageTopic() const;
  std::string getInfoTopic() const;

  void shutdown();
  
private:
  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

} //namespace image_transport

#endif
