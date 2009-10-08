#ifndef IMAGE_TRANSPORT_RAW_PUBLISHER_H
#define IMAGE_TRANSPORT_RAW_PUBLISHER_H

#include "image_transport/simple_publisher_plugin.h"

namespace image_transport {

/**
 * \brief The default PublisherPlugin.
 *
 * RawPublisher is a simple wrapper for ros::Publisher, publishing unaltered Image
 * messages on the base topic.
 */
class RawPublisher : public SimplePublisherPlugin<sensor_msgs::Image>
{
public:
  virtual ~RawPublisher() {}

  virtual std::string getTransportName() const
  {
    return "raw";
  }

protected:
  virtual void publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
  {
    publish_fn(message);
  }

  virtual std::string getTopicToAdvertise(const std::string& base_topic) const
  {
    return base_topic;
  }
};

} //namespace image_transport

#endif
