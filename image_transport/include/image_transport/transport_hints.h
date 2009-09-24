#ifndef IMAGE_TRANSPORT_TRANSPORT_HINTS_H
#define IMAGE_TRANSPORT_TRANSPORT_HINTS_H

#include <ros/ros.h>

namespace image_transport {

/**
 * \brief Stores transport settings for an image topic subscription.
 */
class TransportHints
{
public:
  /**
   * \brief Constructor.
   *
   * The default transport can be overridden by setting a certain parameter to the
   * name of the desired transport. By default this parameter is named "image_transport"
   * in the node's local namespace. For consistency across ROS applications, the
   * name of this parameter should not be changed without good reason.
   *
   * @param default_transport Preferred transport to use
   * @param ros_hints Hints to pass through to ROS subscriptions
   * @param parameter_nh Node handle to use when looking up the transport parameter.
   * Defaults to the local namespace.
   * @param parameter_name The name of the transport parameter
   */
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
