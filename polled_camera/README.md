## `polled_camera` Overview

`polled_camera` defines the ROS interface that client nodes use to request images from a polling camera driver node (e.g. [prosilica_camera](https://index.ros.org/p/prosilica_camera/)). The protocol is:

## Build Status

| ROS2 Distro | Build Status | Package build |
|:---------:|:----:|:----------:|
| Rolling | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__polled_camera__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__polled_camera__ubuntu_resolute_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__polled_camera__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__polled_camera__ubuntu_resolute_amd64__binary/) |
| Lyrical | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Ldev__polled_camera__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__polled_camera__ubuntu_resolute_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__polled_camera__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__polled_camera__ubuntu_resolute_amd64__binary/) |
| Kilted | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kdev__polled_camera__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__polled_camera__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__polled_camera__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__polled_camera__ubuntu_noble_amd64__binary/) |
| Jazzy | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__polled_camera__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__polled_camera__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__polled_camera__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__polled_camera__ubuntu_noble_amd64__binary/) |
| Humble | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__polled_camera__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__polled_camera__ubuntu_jammy_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__polled_camera__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__polled_camera__ubuntu_jammy_amd64__binary/) |

* The camera driver advertises a service call `<camera>/request_image` of type [polled_camera/GetPolledImage](https://github.com/ros-perception/image_common/blob/rolling/polled_camera/srv/GetPolledImage.srv).
* The client calls the service, specifying an output namespace.
* On receiving a request, the driver captures an image and returns its time stamp in the service response.
* The driver publishes (latching) the [sensor_msgs/Image](http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html) and [sensor_msgs/CameraInfo](http://www.ros.org/doc/api/sensor_msgs/html/msg/CameraInfo.html) to `<response_namespace>/image_raw` and `<response_namespace>/camera_info`.
* Clients subscribe to the response topics just like any other camera image stream.
* When a disconnection causes the number of subscribers to drop to zero, the publisher shuts down.

See the [code API](http://www.ros.org/doc/api/polled_camera/html/) (unstable) for more information on writing polled camera drivers or clients.

There are several reasons for publishing images to topics instead of returning them from the service call:

* **Consistency:** Client-side processing is always performed in an image callback, regardless of what type of camera is used.
* **Visibility:** Topics can be monitored in [image_view](https://index.ros.org/p/image_view/) or [rviz](https://index.ros.org/p/rviz/).
* **Reproducibility:** Topics can be [bagged](https://index.ros.org/p/rosbag2/) and played back later.
* **Compression:** Can take advantage of [image_transport](https://index.ros.org/p/image_transport/) to compress images, if desired.

## Nodes

### poller

Continually requests images from a polled camera at a specified rate in Hz, making the camera look like it is capturing continuously.

Usage:

```bash
# Poll "my_camera" at 5 Hz, publishing in namespace my_polled_output/.
$ poller 5 camera:=my_camera output:=my_polled_output
```

Published Topics
* `<output>/image_raw` ([sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg))
  Image topic, actually published by the camera driver. output should be remapped by the user.
* `<output>/camera_info` ([sensor_msgs/CameraInfo](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CameraInfo.msg))
  Info topic, actually published by the camera driver. output should be remapped by the user.
Services Called
* `<camera>/request_image` ([polled_camera/GetPolledImage](https://github.com/ros-perception/image_common/blob/rolling/polled_camera/srv/GetPolledImage.srv))
The camera driver's polled image service. camera should be remapped by the user to the camera namespace.
