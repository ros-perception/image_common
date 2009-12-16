#include "polled_camera/publication_server.h"
#include <image_transport/image_transport.h>

namespace polled_camera {

class PublicationServer::Impl
{
public:
  ros::ServiceServer srv_server_;
  DriverCallback driver_cb_;
  ros::VoidPtr tracked_object_;
  image_transport::ImageTransport it_;
  std::map<std::string, image_transport::CameraPublisher> client_map_;
  bool unadvertised_;
  double constructed_;
  
  Impl(const ros::NodeHandle& nh)
    : it_(nh),
      unadvertised_(false),
      constructed_(ros::WallTime::now().toSec())
  {
  }
  
  ~Impl()
  {
    if (ros::WallTime::now().toSec() - constructed_ < 0.001)
      ROS_WARN("PublicationServer destroyed immediately after creation. Did you forget to store the handle?");
    unadvertise();
  }

  bool isValid() const
  {
    return !unadvertised_;
  }

  void unadvertise()
  {
    if (!unadvertised_) {
      unadvertised_ = true;
      srv_server_.shutdown();
      client_map_.clear();
    }
  }

  bool requestCallback(polled_camera::GetPolledImage::Request& req,
                       polled_camera::GetPolledImage::Response& rsp)
  {
    std::string image_topic = req.response_namespace + "/image_raw";
    image_transport::CameraPublisher& pub = client_map_[image_topic];
    if (!pub) {
      // Create a latching camera publisher.
      //typedef image_transport::SubscriberStatusCallback StatusCallback;
      //StatusCallback disconnectCb = boost::bind(&Impl::disconnectCallback, this, _1);
      pub = it_.advertiseCamera(image_topic, 1, image_transport::SubscriberStatusCallback(),
                                boost::bind(&Impl::disconnectCallback, this, _1),
                                ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
                                ros::VoidPtr(), true /*latch*/);
      ROS_INFO("Advertising %s", pub.getTopic().c_str());
    }

    sensor_msgs::Image image;
    sensor_msgs::CameraInfo info;
    /// @todo Check tracked_object before calling
    bool success = driver_cb_(req, image, info);
    if (success) {
      rsp.stamp = image.header.stamp;
      pub.publish(image, info);
    }
    return success;
  }

  void disconnectCallback(const image_transport::SingleSubscriberPublisher& ssp)
  {
    // Shut down the publication when the subscription count drops to zero.
    if (ssp.getNumSubscribers() == 0) {
      ROS_INFO("Shutting down %s", ssp.getTopic().c_str());
      client_map_.erase(ssp.getTopic());
    }
  }
};

PublicationServer::PublicationServer(const std::string& service, ros::NodeHandle& nh,
                                     const DriverCallback& cb, const ros::VoidPtr& tracked_object)
  : impl_(new Impl(nh))
{
  impl_->driver_cb_ = cb;
  impl_->tracked_object_ = tracked_object;
  impl_->srv_server_ = nh.advertiseService<>(service, &Impl::requestCallback, impl_);
}

void PublicationServer::shutdown()
{
  if (impl_) impl_->unadvertise();
}

std::string PublicationServer::getService() const
{
  if (impl_) return impl_->srv_server_.getService();
  return std::string();
}

PublicationServer::operator void*() const
{
  return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0;
}

PublicationServer advertise(ros::NodeHandle& nh, const std::string& service,
                            const PublicationServer::DriverCallback& cb,
                            const ros::VoidPtr& tracked_object)
{
  return PublicationServer(service, nh, cb, tracked_object);
}

} //namespace polled_camera
