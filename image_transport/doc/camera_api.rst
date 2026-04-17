Camera API
==========

Camera drivers typically publish both an ``Image`` and a ``CameraInfo`` topic.
``image_transport`` provides helper classes that publish and subscribe to these
two topics together, keeping them synchronised.

CameraPublisher
---------------

:cpp:class:`image_transport::CameraPublisher` publishes ``Image`` and
``CameraInfo`` on a pair of synchronised topics derived from a single base
topic name.

See the full :ref:`exhale_class_classimage__transport_1_1CameraPublisher` API reference.

CameraSubscriber
----------------

:cpp:class:`image_transport::CameraSubscriber` subscribes to both the
``Image`` and ``CameraInfo`` topics and delivers them via a single callback.

See the full :ref:`exhale_class_classimage__transport_1_1CameraSubscriber` API reference.

Topic Naming Convention
-----------------------

Given a base topic name (e.g. ``/camera/image_raw``), ``CameraPublisher``
automatically creates the matching camera info topic by appending
``/../camera_info``:

.. code-block:: text

   /camera/image_raw        ← image topic
   /camera/camera_info      ← camera info topic (sibling)

The sibling topic name is computed by
:cpp:func:`image_transport::getCameraInfoTopic`.

Usage Example
-------------

**Publisher side (camera driver)**

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   #include <image_transport/image_transport.hpp>
   #include <sensor_msgs/msg/camera_info.hpp>

   class CameraDriver : public rclcpp::Node
   {
   public:
     CameraDriver() : Node("camera_driver")
     {
       it_ = std::make_shared<image_transport::ImageTransport>(
         shared_from_this());
       cam_pub_ = it_->advertiseCamera("camera/image_raw", 1);
     }

     void publishFrame(
       sensor_msgs::msg::Image::SharedPtr image,
       sensor_msgs::msg::CameraInfo::SharedPtr info)
     {
       cam_pub_.publish(image, info);
     }

   private:
     std::shared_ptr<image_transport::ImageTransport> it_;
     image_transport::CameraPublisher cam_pub_;
   };

**Subscriber side**

.. code-block:: cpp

   it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
   cam_sub_ = it_->subscribeCamera(
     "camera/image_raw", 1,
     [this](
       const sensor_msgs::msg::Image::ConstSharedPtr & img,
       const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
     {
       RCLCPP_INFO(get_logger(), "fx = %f", info->k[0]);
     });
