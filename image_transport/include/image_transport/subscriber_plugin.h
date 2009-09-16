#ifndef IMAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H
#define IMAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/noncopyable.hpp>

namespace image_transport {

/**
 * \brief Base class for plugins to ImageSubscriber.
 */
class SubscriberPlugin : boost::noncopyable
{
public:
  typedef boost::function<void(const sensor_msgs::ImageConstPtr&)> Callback;
  
  virtual ~SubscriberPlugin() {}

  /**
   * \brief Subscribe to an image transport topic.
   */
  virtual void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                         const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                         const ros::VoidPtr& tracked_object,
                         const ros::TransportHints& transport_hints) = 0;
  
  virtual std::string getTopic() const = 0;

  /**
   * \brief Unsubscribe the callback associated with this SubscriberPlugin.
   */
  virtual void shutdown() = 0;

  /**
   * \brief Return the lookup name of the SubscriberPlugin associated with a specific
   * transport identifier.
   */
  static std::string getLookupName(const std::string& transport_type)
  {
    return transport_type + "_sub";
  }
};

} //namespace image_transport

#endif
