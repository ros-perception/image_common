#ifndef IMAGE_TRANSPORT_RAW_PUBLISHER_H
#define IMAGE_TRANSPORT_RAW_PUBLISHER_H

#include "image_transport/publisher_plugin.h"

namespace image_transport {

/**
 * \brief The default PublisherPlugin.
 *
 * RawPublisher is a simple wrapper for ros::Publisher, publishing unaltered Image
 * messages on the base topic.
 */
class RawPublisher : public PublisherPlugin
{
public:
  virtual ~RawPublisher();

  virtual std::string getTransportName() const;

  virtual uint32_t getNumSubscribers() const;
  virtual std::string getTopic() const;

  virtual void publish(const sensor_msgs::Image& message) const;

  virtual void shutdown();

protected:
  ros::Publisher pub_;

  virtual void advertiseImpl(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size,
                             const SubscriberStatusCallback& connect_cb,
                             const SubscriberStatusCallback& disconnect_cb,
                             const ros::VoidPtr& tracked_object, bool latch);

  ros::SubscriberStatusCallback bindCB(const SubscriberStatusCallback& user_cb);
  
  void subscriberCB(const ros::SingleSubscriberPublisher& ros_pub,
                    const SubscriberStatusCallback& user_cb);
};

} //namespace image_transport

#endif
