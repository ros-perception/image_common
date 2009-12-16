#ifndef POLLED_CAMERA_PUBLICATION_SERVER_H
#define POLLED_CAMERA_PUBLICATION_SERVER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "polled_camera/GetPolledImage.h"

namespace polled_camera {

class PublicationServer
{
public:
  typedef boost::function<bool (polled_camera::GetPolledImage::Request&,
                                sensor_msgs::Image&,
                                sensor_msgs::CameraInfo&)> DriverCallback;
  
  PublicationServer() {}
  
  void shutdown();

  std::string getService() const;

  operator void*() const;
  bool operator< (const PublicationServer& rhs) const { return impl_ <  rhs.impl_; }
  bool operator==(const PublicationServer& rhs) const { return impl_ == rhs.impl_; }
  bool operator!=(const PublicationServer& rhs) const { return impl_ != rhs.impl_; }

private:
  PublicationServer(const std::string& service, ros::NodeHandle& nh,
                    const DriverCallback& cb, const ros::VoidPtr& tracked_object);

  class Impl;

  boost::shared_ptr<Impl> impl_;

  friend
  PublicationServer advertise(ros::NodeHandle&, const std::string&, const DriverCallback&, const ros::VoidPtr&);
};

PublicationServer advertise(ros::NodeHandle& nh, const std::string& service,
                            const PublicationServer::DriverCallback& cb,
                            const ros::VoidPtr& tracked_object = ros::VoidPtr());

template<class T>
PublicationServer advertise(ros::NodeHandle& nh, const std::string& service,
                            bool(T::*fp)(polled_camera::GetPolledImage::Request&,
                                         sensor_msgs::Image&, sensor_msgs::CameraInfo&),
                            T* obj)
{
  return advertise(nh, service, boost::bind(fp, obj, _1, _2, _3));
}

template<class T>
PublicationServer advertise(ros::NodeHandle& nh, const std::string& service,
                            bool(T::*fp)(polled_camera::GetPolledImage::Request&,
                                         sensor_msgs::Image&, sensor_msgs::CameraInfo&),
                            const boost::shared_ptr<T>& obj)
{
  return advertise(nh, service, boost::bind(fp, obj.get(), _1, _2, _3), obj);
}

} //namespace polled_camera

#endif
