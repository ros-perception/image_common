#ifndef IMAGE_TRANSPORT_TRANSPORT_HINTS_H
#define IMAGE_TRANSPORT_TRANSPORT_HINTS_H

#include <ros/ros.h>

namespace image_transport {

class TransportHints
{
public:
  TransportHints(const std::string& default_transport = "raw",
                 const ros::TransportHints& ros_hints = ros::TransportHints(),
                 const ros::NodeHandle& parameter_nh = ros::NodeHandle("~"),
                 const std::string& parameter_name = "image_transport")
    : ros_hints_(ros_hints)
  {
    parameter_nh.param(parameter_name, transport_, default_transport);
  }

  const std::string& getTransport() const
  {
    return transport_;
  }

  const ros::TransportHints& getRosHints() const
  {
    return ros_hints_;
  }
  
private:
  std::string transport_;
  ros::TransportHints ros_hints_;
};

} //namespace image_transport

#endif
