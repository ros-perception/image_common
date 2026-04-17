Overview
========

``camera_info_manager`` provides a C++ class that manages
``sensor_msgs/msg/CameraInfo`` for camera drivers. It handles:

- **Loading** calibration data from a file or ``package://`` URL on first
  access (lazy loading).
- **Saving** calibration data to a file in response to a
  ``sensor_msgs/srv/SetCameraInfo`` service call.
- **Advertising** the ``set_camera_info`` service so calibration tools can
  push new calibration data to a running camera driver.

Architecture
------------

A camera driver creates a ``CameraInfoManager`` at startup, passing node
interfaces and a calibration URL. The manager advertises the
``set_camera_info`` service and lazy-loads calibration data the first time
``getCameraInfo()`` or ``isCalibrated()`` is called.

.. code-block:: none

   Camera Driver Node
   ├── CameraInfoManager
   │   ├── Reads calibration URL → loads CameraInfo from file/package
   │   ├── Serves  ~/set_camera_info  → saves new calibration to file
   │   └── Returns CameraInfo via getCameraInfo()
   └── Publishes Image + CameraInfo on every frame

Calibration URL Schemes
-----------------------

``camera_info_manager`` supports two URL schemes for locating calibration
files:

``file://``
    Absolute path on the local filesystem.

    .. code-block:: none

       file:///home/user/calibrations/camera.yaml

``package://``
    Path relative to the install prefix of a ROS 2 package.

    .. code-block:: none

       package://my_camera_driver/config/calibration.yaml

URL Variable Substitution
~~~~~~~~~~~~~~~~~~~~~~~~~

URLs may contain the following variables (resolved in a single pass):

- ``${NAME}`` — replaced with the camera name set at construction time or
  via ``setCameraName()``.
- ``${ROS_HOME}`` — replaced with the value of the ``$ROS_HOME`` environment
  variable, or ``~/.ros`` if unset.

Default URL (when none is provided):

.. code-block:: none

   file://${ROS_HOME}/camera_info/${NAME}.yaml

Camera Name
-----------

The camera name is used for two purposes:

1. Substituted for ``${NAME}`` in the calibration URL.
2. Compared against the camera name stored inside the calibration file — a
   warning is logged if they differ.

A valid camera name consists of alphanumeric characters and underscores.
For uniqueness, consider encoding a GUID or make/model/serial combination.

Lazy Loading
------------

No I/O is performed in the constructor.  Calibration data are read on the
first call to ``getCameraInfo()``, ``isCalibrated()``, or an explicit
``loadCameraInfo()``.  Call ``loadCameraInfo()`` early in the driver's
``on_activate()`` or equivalent lifecycle callback to surface load errors
before streaming begins.

ROS 2 Service
-------------

The manager advertises ``~/set_camera_info`` (relative to the node namespace
unless overridden).  Calibration tools such as
``camera_calibration`` send new calibration data via this service; the manager
saves the data to the configured URL and updates the in-memory CameraInfo.

Lifecycle Node Support
----------------------

Pass a ``rclcpp_lifecycle::LifecycleNode *`` to the deprecated constructor, or
extract node interfaces from the lifecycle node and use the preferred
interface-based constructor.

.. code-block:: cpp

   auto cim = std::make_shared<camera_info_manager::CameraInfoManager>(
     node->get_node_base_interface(),
     node->get_node_services_interface(),
     node->get_node_logging_interface(),
     "my_camera",
     "package://my_driver/config/camera.yaml");

Dependencies
------------

- ``rclcpp`` — node interfaces and service server.
- ``rclcpp_lifecycle`` — lifecycle node support (deprecated constructor path).
- ``sensor_msgs`` — ``CameraInfo`` and ``SetCameraInfo`` types.
- ``camera_calibration_parsers`` — YAML / INI calibration file I/O.
- ``ament_index_cpp`` — resolves ``package://`` URLs to filesystem paths.
- ``rcpputils`` — utility helpers.
