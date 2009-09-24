#ifndef IMAGE_TRANSPORT_RAW_SUBSCRIBER_H
#define IMAGE_TRANSPORT_RAW_SUBSCRIBER_H

#include "image_transport/simple_subscriber_plugin.h"

namespace image_transport {

/**
 * \brief The default SubscriberPlugin.
 *
 * RawSubscriber is a simple wrapper for ros::Subscriber which listens for Image messages
 * and passes them through to the callback.
 */
class RawSubscriber : public SimpleSubscriberPlugin<sensor_msgs::Image>
{
public:
  virtual ~RawSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "raw";
  }

protected:
  virtual void internalCallback(const sensor_msgs::ImageConstPtr& message, const Callback& user_cb)
  {
    user_cb(message);
  }

  virtual std::string getTopicToSubscribe(const std::string& base_topic) const
  {
    return base_topic;
  }
};

} //namespace image_transport

#endif
