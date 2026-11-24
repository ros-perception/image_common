Using camera_info_manager in a camera driver (C++)
==================================================

Overview
--------

This tutorial shows how to provide calibration information while publishing
images. It creates a minimal camera driver which:

- publishes an image together with its ``CameraInfo`` using
  ``image_transport``,
- loads the calibration of the camera from a URL given as a parameter,
- lets a calibration tool store a new calibration with the
  ``set_camera_info`` service.

To keep the example short, the driver does not read a real camera: it
publishes black images.

Prerequisites
-------------

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_
and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Write the driver
-------------------

Let's assume you have already created an empty ROS 2 package for C++ named
``camera_tutorial``. Create a new file ``src/camera_driver.cpp`` inside your
package, with the following content:

.. code-block:: cpp

   #include <chrono>
   #include <memory>
   #include <string>

   #include "camera_info_manager/camera_info_manager.hpp"
   #include "image_transport/image_transport.hpp"
   #include "rclcpp/rclcpp.hpp"
   #include "sensor_msgs/image_encodings.hpp"
   #include "sensor_msgs/msg/camera_info.hpp"
   #include "sensor_msgs/msg/image.hpp"

   class CameraDriver : public rclcpp::Node
   {
   public:
     CameraDriver()
     : Node("camera_driver")
     {
       const std::string camera_name = declare_parameter("camera_name", "my_camera");
       const std::string camera_info_url = declare_parameter("camera_info_url", "");
       frame_id_ = declare_parameter("frame_id", "camera_optical_frame");

       cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(
         get_node_base_interface(),
         get_node_services_interface(),
         get_node_logging_interface(),
         camera_name,
         camera_info_url,
         rclcpp::SystemDefaultsQoS(),
         "~");

       if (!cinfo_->validateURL(camera_info_url)) {
         RCLCPP_ERROR(
           get_logger(), "Invalid camera_info_url: %s", camera_info_url.c_str());
       } else if (!cinfo_->loadCameraInfo(camera_info_url)) {
         RCLCPP_WARN(get_logger(), "The camera is not calibrated");
       }

       publisher_ = image_transport::create_camera_publisher(
         *this, "~/image_raw", rclcpp::SystemDefaultsQoS());

       timer_ = create_wall_timer(
         std::chrono::milliseconds(100), [this]() {publish_frame();});
     }

   private:
     void publish_frame()
     {
       // A real driver would get the image from the camera here
       sensor_msgs::msg::Image image;
       image.header.stamp = now();
       image.header.frame_id = frame_id_;
       image.width = 640;
       image.height = 480;
       image.encoding = sensor_msgs::image_encodings::MONO8;
       image.step = image.width;
       image.data.resize(image.step * image.height, 0);

       sensor_msgs::msg::CameraInfo info = cinfo_->getCameraInfo();
       info.header = image.header;

       publisher_.publish(image, info);
     }

     std::string frame_id_;
     std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
     image_transport::CameraPublisher publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
   };

   int main(int argc, char ** argv)
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<CameraDriver>());
     rclcpp::shutdown();
     return 0;
   }

2. Examine the code
-------------------

.. code-block:: cpp

       const std::string camera_name = declare_parameter("camera_name", "my_camera");
       const std::string camera_info_url = declare_parameter("camera_info_url", "");
       frame_id_ = declare_parameter("frame_id", "camera_optical_frame");

``camera_info_manager`` does not read parameters by itself, so the driver
declares the ones it needs. ``camera_name`` and ``camera_info_url`` are the
names that most of the camera drivers use.

.. code-block:: cpp

       cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(
         get_node_base_interface(),
         get_node_services_interface(),
         get_node_logging_interface(),
         camera_name,
         camera_info_url,
         rclcpp::SystemDefaultsQoS(),
         "~");

Create the ``CameraInfoManager``. It needs three interfaces of the node, the
name of the camera and the URL of the calibration. The constructor advertises
the ``set_camera_info`` service, and it does not load the calibration yet.

The last two arguments are optional: the QoS of the service and its namespace.
With ``"~"`` the service is advertised inside the namespace of the node, as
``/camera_driver/set_camera_info``. By default it would be advertised as
``/set_camera_info``, which is the same name for every driver.

.. code-block:: cpp

       if (!cinfo_->validateURL(camera_info_url)) {
         RCLCPP_ERROR(
           get_logger(), "Invalid camera_info_url: %s", camera_info_url.c_str());
       } else if (!cinfo_->loadCameraInfo(camera_info_url)) {
         RCLCPP_WARN(get_logger(), "The camera is not calibrated");
       }

The calibration would be loaded automatically the first time it is needed.
Loading it explicitly at startup reports the problems earlier.

- ``validateURL()`` checks the syntax of the URL. Only ``file://`` and
  ``package://`` URLs are supported. The empty URL is valid too, and it stands
  for ``file://${ROS_HOME}/camera_info/${NAME}.yaml``.
- ``loadCameraInfo()`` returns ``false`` if the calibration cannot be loaded.
  This is not a fatal error: a camera can be used without calibration, and it
  is the normal situation before calibrating it for the first time.

.. code-block:: cpp

       publisher_ = image_transport::create_camera_publisher(
         *this, "~/image_raw", rclcpp::SystemDefaultsQoS());

Create an ``image_transport`` camera publisher, which publishes the image and
its ``CameraInfo`` in a pair of synchronized topics:
``/camera_driver/image_raw`` and ``/camera_driver/camera_info``.

.. code-block:: cpp

       sensor_msgs::msg::CameraInfo info = cinfo_->getCameraInfo();
       info.header = image.header;

       publisher_.publish(image, info);

For every image, get the current ``CameraInfo`` from the manager.
``getCameraInfo()`` returns a copy of the calibration and it does not fill in
the header, so copy the time stamp and the frame ID from the image. Then
publish both messages together.

Ask for the ``CameraInfo`` every time, instead of storing a copy: the
calibration changes when the ``set_camera_info`` service is called.

3. Update package.xml
---------------------

Add the dependencies to your ``package.xml``:

.. code-block:: xml

   <depend>camera_info_manager</depend>
   <depend>image_transport</depend>
   <depend>rclcpp</depend>
   <depend>sensor_msgs</depend>

4. Add the node to CMakeLists.txt
---------------------------------

.. code-block:: cmake

   find_package(ament_cmake REQUIRED)
   find_package(camera_info_manager REQUIRED)
   find_package(image_transport REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(sensor_msgs REQUIRED)

   add_executable(camera_driver src/camera_driver.cpp)
   target_link_libraries(camera_driver PRIVATE
     camera_info_manager::camera_info_manager
     image_transport::image_transport
     rclcpp::rclcpp
     ${sensor_msgs_TARGETS})

   install(TARGETS camera_driver
     DESTINATION lib/${PROJECT_NAME})

   ament_package()

5. Build and run
----------------

Build your package from the root of your workspace and source it:

.. code-block:: console

   $ colcon build --packages-select camera_tutorial
   $ source install/setup.bash

Run the driver, storing the calibration in ``/tmp``:

.. code-block:: console

   $ ros2 run camera_tutorial camera_driver --ros-args -p camera_info_url:='file:///tmp/${NAME}.yaml'
   [INFO] [camera_driver]: camera calibration URL: file:///tmp/my_camera.yaml
   [ERROR] [camera_calibration_parsers]: Unable to open camera calibration file [/tmp/my_camera.yaml]
   [WARN] [camera_driver]: Camera calibration file /tmp/my_camera.yaml not found
   [WARN] [camera_driver]: The camera is not calibrated

The time stamps of the log messages are omitted here.

The ``${NAME}`` variable of the URL has been replaced by the name of the
camera. The file does not exist yet, so the camera is not calibrated and the
``CameraInfo`` messages are empty:

.. code-block:: console

   $ ros2 topic echo --once /camera_driver/camera_info --field k
   [0. 0. 0. 0. 0. 0. 0. 0. 0.]
   ---

6. Set the calibration
----------------------

A calibration tool, like ``cameracalibrator`` from the ``camera_calibration``
package, sends the calibration to the driver with the ``set_camera_info``
service. You can do the same from another terminal:

.. code-block:: console

   $ ros2 service call /camera_driver/set_camera_info sensor_msgs/srv/SetCameraInfo "{camera_info: {width: 640, height: 480, distortion_model: plumb_bob, d: [0.0, 0.0, 0.0, 0.0, 0.0], k: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0], r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], p: [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]}}"

   response:
   sensor_msgs.srv.SetCameraInfo_Response(success=True, status_message='')

The driver uses the new calibration immediately:

.. code-block:: console

   $ ros2 topic echo --once /camera_driver/camera_info --field k
   [500.   0. 320.   0. 500. 240.   0.   0.   1.]
   ---

It has also saved the calibration to ``/tmp/my_camera.yaml``, so it is loaded
the next time the driver starts:

.. code-block:: console

   $ cat /tmp/my_camera.yaml
   image_width: 640
   image_height: 480
   camera_name: my_camera
   camera_matrix:
     rows: 3
     cols: 3
     data: [500, 0, 320, 0, 500, 240, 0, 0, 1]
   distortion_model: plumb_bob
   distortion_coefficients:
     rows: 1
     cols: 5
     data: [0, 0, 0, 0, 0]
   rectification_matrix:
     rows: 3
     cols: 3
     data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
   projection_matrix:
     rows: 3
     cols: 4
     data: [500, 0, 320, 0, 0, 500, 240, 0, 0, 0, 1, 0]
   binning_x: 0
   binning_y: 0
   roi:
     x_offset: 0
     y_offset: 0
     height: 0
     width: 0
     do_rectify: false

To distribute a calibration with a package, install the file in the share
directory of the package and use a ``package://`` URL, for example
``package://camera_tutorial/config/my_camera.yaml``.
