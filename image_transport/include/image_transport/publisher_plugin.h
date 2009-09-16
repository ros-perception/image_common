#ifndef IMAGE_TRANSPORT_PUBLISHER_PLUGIN_H
#define IMAGE_TRANSPORT_PUBLISHER_PLUGIN_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/noncopyable.hpp>

namespace image_transport {

/**
 * \brief Base class for plugins to ImagePublisher.
 *
 * @todo overloads of advertise? may be unnecessary here.
 */
class PublisherPlugin : boost::noncopyable
{
public:
  virtual ~PublisherPlugin() {}

  /**
   * \brief Get a string identifier for the transport type provided by
   * this plugin.
   */
  virtual std::string getTransportType() const = 0;

  /**
   * \brief Given a base topic, return the default subtopic for this transport.
   */
  virtual std::string getDefaultTopic(const std::string& base_topic) const
  {
    return base_topic + "_" + getTransportType();
  }

  /**
   * \brief Advertise a topic.
   */
  virtual void advertise(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                         const ros::SubscriberStatusCallback& connect_cb,
                         const ros::SubscriberStatusCallback& disconnect_cb,
                         const ros::VoidPtr& tracked_object, bool latch) = 0;

  /**
   * \brief Returns the number of subscribers that are currently connected to
   * this PublisherPlugin.
   */
  virtual uint32_t getNumSubscribers() const = 0;

  /**
   * \brief Returns the topic that this PublisherPlugin will publish on.
   */
  virtual std::string getTopic() const = 0;

  /**
   * \brief Publish an image using the transport associated with this PublisherPlugin.
   */
  virtual void publish(const sensor_msgs::Image& message) const = 0;

  /**
   * \brief Publish an image using the transport associated with this PublisherPlugin.
   */
  virtual void publish(const sensor_msgs::ImageConstPtr& message) const
  {
    publish(*message);
  }

  /**
   * \brief Shutdown any advertisements associated with this PublisherPlugin.
   */
  virtual void shutdown() = 0;

  /**
   * \brief Return the lookup name of the PublisherPlugin associated with a specific
   * transport identifier.
   */
  static std::string getLookupName(const std::string& transport_type)
  {
    return transport_type + "_pub";
  }
};

} //namespace image_transport

#endif
