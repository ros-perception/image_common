API Reference
=============

CameraInfoManager
-----------------

:cpp:class:`camera_info_manager::CameraInfoManager` is the main class.
Camera drivers create one instance per camera and use it to serve calibration
data.

See the full :ref:`exhale_class_classcamera__info__manager_1_1CameraInfoManager`
API reference.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Method
     - Description
   * - ``getCameraInfo()``
     - Returns a copy of the current ``CameraInfo``, loading it first if
       needed. The caller must fill in the header.
   * - ``isCalibrated()``
     - Returns whether calibration data are available, loading them first if
       needed.
   * - ``loadCameraInfo(url)``
     - Sets a new calibration URL and loads it immediately.
   * - ``setCameraName(cname)``
     - Sets a new camera name. The calibration is loaded again the next time
       it is needed.
   * - ``setCameraInfo(camera_info)``
     - Sets the current ``CameraInfo``, without saving it.
   * - ``validateURL(url)``
     - Checks whether the syntax of a URL is supported.
   * - ``resolveURL(url, cname)``
     - Returns the URL with its substitution variables resolved.

Type Aliases
------------

The following type aliases are defined in the ``camera_info_manager`` namespace:

- ``CameraInfo`` — alias for ``sensor_msgs::msg::CameraInfo``.
- ``SetCameraInfo`` — alias for ``sensor_msgs::srv::SetCameraInfo``.

Usage Examples
--------------

See the :doc:`tutorial` for a complete camera driver.

**Creating the manager**

.. code-block:: cpp

   #include "camera_info_manager/camera_info_manager.hpp"

   // "node" is a rclcpp::Node or a rclcpp_lifecycle::LifecycleNode
   auto cim = std::make_shared<camera_info_manager::CameraInfoManager>(
     node->get_node_base_interface(),
     node->get_node_services_interface(),
     node->get_node_logging_interface(),
     "my_camera",                             // camera name
     "package://my_driver/config/cam.yaml");  // calibration URL

**Using a different QoS or namespace for the set_camera_info service**

.. code-block:: cpp

   // Advertises "~/set_camera_info" instead of "/set_camera_info"
   auto cim = std::make_shared<camera_info_manager::CameraInfoManager>(
     node->get_node_base_interface(),
     node->get_node_services_interface(),
     node->get_node_logging_interface(),
     "my_camera",
     "package://my_driver/config/cam.yaml",
     rclcpp::SystemDefaultsQoS(),
     "~");

**Publishing the calibration with an image**

.. code-block:: cpp

   sensor_msgs::msg::CameraInfo info = cim->getCameraInfo();
   // getCameraInfo() does not fill in the header
   info.header = image.header;
   camera_publisher.publish(image, info);

**Changing the calibration URL at runtime**

.. code-block:: cpp

   // Reload from a different file without restarting the node:
   if (!cim->validateURL(new_url)) {
     RCLCPP_ERROR(node->get_logger(), "Invalid calibration URL: %s", new_url.c_str());
   } else if (!cim->loadCameraInfo(new_url)) {
     RCLCPP_WARN(node->get_logger(), "No calibration found in %s", new_url.c_str());
   }

**Checking calibration status**

.. code-block:: cpp

   if (cim->isCalibrated()) {
     auto ci = cim->getCameraInfo();
     RCLCPP_INFO(node->get_logger(), "fx = %f", ci.k[0]);
   } else {
     RCLCPP_WARN(node->get_logger(), "Camera not calibrated");
   }
