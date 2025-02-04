## `polled_camera` Overview

`polled_camera` defines the ROS interface that client nodes use to request images from a polling camera driver node (e.g. [prosilica_camera](https://index.ros.org/p/prosilica_camera). The protocol is:
 * The camera driver advertises a service call `<camera>/request_image` of type <<SrvLink(polled_camera/GetPolledImage)>>.
 * The client calls the service, specifying an output namespace.
 * On receiving a request, the driver captures an image and returns its time stamp in the service response.
 * The driver publishes (latching) the <<MsgLink(sensor_msgs/Image)>> and <<MsgLink(sensor_msgs/CameraInfo)>> to `<response_namespace>/image_raw` and `<response_namespace>/camera_info`.
 * Clients subscribe to the response topics just like any other camera image stream.
 * When a disconnection causes the number of subscribers to drop to zero, the publisher shuts down.

See the [code API](http://www.ros.org/doc/api/polled_camera/html/) (unstable) for more information on writing polled camera drivers or clients.

There are several reasons for publishing images to topics instead of returning them from the service call:
 * Consistency. Client-side processing is always performed in an image callback, regardless of what type of camera is used.
 * Visibility. Topics can be monitored in [image_view](https://index.ros.org/p/image_view) or [rviz](https://index.ros.org/p/rviz).
 * Reproducibility. Topics can be bagged and played back later.
 * Compression: Can take advantage of [image_transport](https://index.ros.org/p/image_transport) to compressed images, if desired.

## Nodes

### `poller`

Continually requests images from a polled camera at a specified rate in Hz, making the camera look like it is capturing continuously.

Usage:

    # Poll "my_camera" at 5 Hz, publishing in namespace my_polled_output/.
    $ poller 5 camera:=my_camera output:=my_polled_output

Published Topics
* <output>/image_raw (sensor_msgs/Image)
  Image topic, actually published by the camera driver. output should be remapped by the user.
* <output>/camera_info (sensor_msgs/CameraInfo)
  Info topic, actually published by the camera driver. output should be remapped by the user.
Services Called
* <camera>/request_image (polled_camera/GetPolledImage)
The camera driver's polled image service. camera should be remapped by the user to the camera namespace.
