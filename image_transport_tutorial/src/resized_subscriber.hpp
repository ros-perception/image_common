#include <image_transport/simple_subscriber_plugin.hpp>
#include "image_transport_tutorial/msg/resized_image.hpp"

IMAGE_TRANSPORT_PUBLIC
class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<image_transport_tutorial::ResizedImage>
{
public:
  IMAGE_TRANSPORT_PUBLIC
  virtual ~ResizedSubscriber() {}

  IMAGE_TRANSPORT_PUBLIC
  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  IMAGE_TRANSPORT_PUBLIC
  virtual void internalCallback(const typename image_transport_tutorial::ResizedImage::ConstPtr& message,
                                const Callback& user_cb);
};
