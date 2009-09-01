#include "image_transport/publisher_plugin.h"

namespace image_transport {

class RawPublisher : public PublisherPlugin
{
public:
  virtual ~RawPublisher();

  virtual std::string getTransportType() const;
  virtual std::string getDefaultTopic(const std::string& base_topic) const;

  virtual void advertise(ros::NodeHandle& nh, const std::string& topic,
                         uint32_t queue_size, bool latch);

  virtual uint32_t getNumSubscribers() const;
  virtual std::string getTopic() const;

  virtual void publish(const sensor_msgs::Image& message) const;

  virtual void shutdown();

protected:
  ros::Publisher pub_;
};

} //namespace image_transport
