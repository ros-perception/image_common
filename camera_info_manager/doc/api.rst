API Reference
=============

CameraInfoManager
-----------------

:cpp:class:`camera_info_manager::CameraInfoManager` is the main class.
Camera drivers create one instance per camera and use it to serve calibration
data.

See the full :ref:`exhale_class_classcamera__info__manager_1_1CameraInfoManager`
API reference.

Type Aliases
------------

The following type aliases are defined in the ``camera_info_manager`` namespace:

- ``CameraInfo`` — alias for ``sensor_msgs::msg::CameraInfo``.
- ``SetCameraInfo`` — alias for ``sensor_msgs::srv::SetCameraInfo``.

Usage Example
-------------

**Minimal camera driver**

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   #include <camera_info_manager/camera_info_manager.hpp>
   #include <image_transport/image_transport.hpp>

   class MyCameraDriver : public rclcpp::Node
   {
   public:
     MyCameraDriver()
     : Node("my_camera")
     {
       cim_ = std::make_shared<camera_info_manager::CameraInfoManager>(
         get_node_base_interface(),
         get_node_services_interface(),
         get_node_logging_interface(),
         "my_camera",                           // camera name
         "package://my_driver/config/cam.yaml"  // calibration URL
       );

       // Eagerly load calibration so errors surface before streaming starts.
       if (!cim_->loadCameraInfo("")) {
         RCLCPP_WARN(get_logger(), "No calibration loaded");
       }

       auto it = image_transport::ImageTransport(shared_from_this());
       cam_pub_ = it.advertiseCamera("image_raw", 1);

       timer_ = create_wall_timer(
         std::chrono::milliseconds(33),
         std::bind(&MyCameraDriver::publishFrame, this));
     }

   private:
     void publishFrame()
     {
       auto image = std::make_shared<sensor_msgs::msg::Image>();
       // ... fill image from hardware ...

       auto ci = std::make_shared<sensor_msgs::msg::CameraInfo>(
         cim_->getCameraInfo());
       ci->header = image->header;
       cam_pub_.publish(image, ci);
     }

     std::shared_ptr<camera_info_manager::CameraInfoManager> cim_;
     image_transport::CameraPublisher cam_pub_;
     rclcpp::TimerBase::SharedPtr timer_;
   };

**Changing the calibration URL at runtime**

.. code-block:: cpp

   // Reload from a different file without restarting the node:
   cim_->loadCameraInfo("file:///tmp/new_calibration.yaml");

**Checking calibration status**

.. code-block:: cpp

   if (cim_->isCalibrated()) {
     auto ci = cim_->getCameraInfo();
     RCLCPP_INFO(get_logger(), "fx = %f", ci.k[0]);
   } else {
     RCLCPP_WARN(get_logger(), "Camera not calibrated");
   }
