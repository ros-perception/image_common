#include <image_transport/simple_subscriber_plugin.hpp>
#include <image_transport_tutorial/msg/resized_image.hpp>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<image_transport_tutorial::msg::ResizedImage>
{
public:
  virtual ~ResizedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void internalCallback(const typename image_transport_tutorial::msg::ResizedImage::ConstSharedPtr& message,
                                const Callback& user_cb);
};
