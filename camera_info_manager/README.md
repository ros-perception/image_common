# camera_info_manager

This package provides `camera_info_manager::CameraInfoManager`, a C++ class used by camera drivers to manage the camera calibration data (`sensor_msgs/msg/CameraInfo`) required by the ROS image pipeline.

It contains no ROS nodes or command line tools.
For camera drivers written in Python, the [camera_info_manager_py](https://docs.ros.org/en/rolling/p/camera_info_manager_py/) package provides a similar interface.

When instantiated in a camera driver node, the class:

- loads the calibration from a URL the first time it is needed,
- provides the current `CameraInfo` to publish together with every image,
- advertises a `set_camera_info` service, and saves the calibration it receives.

## Usage

```cpp
#include "camera_info_manager/camera_info_manager.hpp"

// In the constructor of the driver node
cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(
  get_node_base_interface(),
  get_node_services_interface(),
  get_node_logging_interface(),
  "my_camera",                                      // camera name
  "package://my_driver/config/my_camera.yaml");     // calibration URL

// For every image
sensor_msgs::msg::CameraInfo info = cinfo_->getCameraInfo();
info.header = image.header;
```

`getCameraInfo()` does not fill in the header: the driver has to set the time stamp and the frame ID, normally to the same values as the corresponding image.

```cmake
find_package(camera_info_manager REQUIRED)
target_link_libraries(my_driver PUBLIC camera_info_manager::camera_info_manager)
```

## ROS API

### Services

- `set_camera_info` ([sensor_msgs/srv/SetCameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/srv/SetCameraInfo.html)): sets the calibration and saves it to the calibration URL.
  The response reports whether the calibration could be saved.

The service name is `<ns>/set_camera_info`, where `ns` is the last argument of the constructor.
It is empty by default, which results in the absolute name `/set_camera_info`.
Pass a namespace (for example `"~"`, or the name of the camera) when the node is in a namespace or when it manages several cameras.

### Parameters

This package does not read ROS parameters.
Where appropriate, we recommend that drivers provide a `camera_info_url` parameter for the URL given to `CameraInfoManager`, and a `camera_name` parameter for the camera name.

## Camera names

The camera name is written when the calibration is saved, and checked when it is loaded: a warning is logged if the name in the file does not match.

A valid name contains any combination of alphabetic, numeric and `_` characters, and case is significant.
The name should be unique for each device, so consider a GUID or a combination of the make, model and serial number.

## Calibration URLs

The location for loading and saving the calibration is expressed as a URL:

- `file:///full/path/to/my_camera.yaml`
- `package://my_package/relative/path/to/my_camera.yaml`, relative to the share directory of the package

The format of the file is selected by its extension: `.yaml` or `.yml` (recommended), or `.ini` (Videre INI format).
See [camera_calibration_parsers](https://docs.ros.org/en/rolling/p/camera_calibration_parsers/) for a description of the formats.

The URL may contain substitution variables:

- `${NAME}` is replaced by the camera name
- `${ROS_HOME}` is replaced by the `ROS_HOME` environment variable, or by `~/.ros` if it is not set

If the URL is empty, `file://${ROS_HOME}/camera_info/${NAME}.yaml` is used.

## Documentation

The full documentation, including a tutorial and the C++ API reference, is in the [doc](https://github.com/ros-perception/image_common/tree/rolling/camera_info_manager/doc) directory, and it is published at [docs.ros.org](https://docs.ros.org/en/rolling/p/camera_info_manager/).
