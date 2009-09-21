#ifndef IMAGE_TRANSPORT_IMAGE_TRANSPORT_H
#define IMAGE_TRANSPORT_IMAGE_TRANSPORT_H

#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/camera_subscriber.h"

namespace image_transport {

class ImageTransport
{
public:
  ImageTransport(const ros::NodeHandle& nh);

  /*!
   * \brief Advertise an image topic, simple version.
   */
  Publisher advertise(const std::string& base_topic, uint32_t queue_size, bool latch = false);

  /*!
   * \brief Advertise an image topic with subcriber status callbacks.
   */
  Publisher advertise(const std::string& base_topic, uint32_t queue_size,
                      const ros::SubscriberStatusCallback& connect_cb,
                      const ros::SubscriberStatusCallback& disconnect_cb = ros::SubscriberStatusCallback(),
                      const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);

  /**
   * \brief Subscribe to an image topic, version for arbitrary boost::function object.
   */
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       const boost::function<void(const sensor_msgs::ImageConstPtr&)>& callback,
                       const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                       const TransportHints& transport_hints = TransportHints());

  /**
   * \brief Subscribe to an image topic, version for bare function.
   */
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(*fp)(const sensor_msgs::ImageConstPtr&),
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size,
                     boost::function<void(const sensor_msgs::ImageConstPtr&)>(fp),
                     ros::VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with bare pointer.
   */
  template<class T>
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(T::*fp)(const sensor_msgs::ImageConstPtr&), T* obj,
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
  }

  /**
   * \brief Subscribe to an image topic, version for class member function with shared_ptr.
   */
  template<class T>
  Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                       void(T::*fp)(const sensor_msgs::ImageConstPtr&),
                       const boost::shared_ptr<T>& obj,
                       const TransportHints& transport_hints = TransportHints())
  {
    return subscribe(base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
  }

  /*!
   * \brief Advertise a synchronized camera raw image + info topic pair.
   */
  CameraPublisher advertiseCamera(const std::string& base_topic, uint32_t queue_size, bool latch = false);

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for arbitrary
   * boost::function object.
   *
   * This version assumes the standard topic naming scheme, where the info topic is
   * named "camera_info" in the same namespace as the base image topic.
   */
  CameraSubscriber subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                   const CameraSubscriber::Callback& callback,
                                   const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                                   const TransportHints& transport_hints = TransportHints());

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for bare function.
   */
  CameraSubscriber subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                   void(*fp)(const sensor_msgs::ImageConstPtr,
                                             const sensor_msgs::CameraInfoConstPtr&),
                                   const TransportHints& transport_hints = TransportHints())
  {
    return subscribeCamera(base_topic, queue_size, CameraSubscriber::Callback(fp), ros::VoidPtr(),
                           transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with bare pointer.
   */
  template<class T>
  CameraSubscriber subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                   void(T::*fp)(const sensor_msgs::ImageConstPtr,
                                                const sensor_msgs::CameraInfoConstPtr&), T* obj,
                                   const TransportHints& transport_hints = TransportHints())
  {
    return subscribeCamera(base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(),
                           transport_hints);
  }

  /**
   * \brief Subscribe to a synchronized image & camera info topic pair, version for class member
   * function with shared_ptr.
   */
  template<class T>
  CameraSubscriber subscribeCamera(const std::string& base_topic, uint32_t queue_size,
                                   void(T::*fp)(const sensor_msgs::ImageConstPtr,
                                                const sensor_msgs::CameraInfoConstPtr&),
                                   const boost::shared_ptr<T>& obj,
                                   const TransportHints& transport_hints = TransportHints())
  {
    return subscribeCamera(base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj,
                           transport_hints);
  }

private:
  ros::NodeHandle nh_;
};

} //namespace image_transport

#endif
