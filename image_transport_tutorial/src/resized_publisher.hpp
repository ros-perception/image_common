#include <image_transport/simple_publisher_plugin.hpp>
#include "image_transport_tutorial/msg/resized_image.hpp"

IMAGE_TRANSPORT_PUBLIC
class ResizedPublisher : public image_transport::SimplePublisherPlugin<image_transport_tutorial::ResizedImage>
{
public:
  IMAGE_TRANSPORT_PUBLIC
  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  IMAGE_TRANSPORT_PUBLIC
  virtual void publish(const sensor_msgs::Image& message,
                       const PublishFn& publish_fn) const;
};
