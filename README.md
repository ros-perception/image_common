# image_transport_py: Python Bindings for ROS2 Image Transport

## Introduction

`image_transport_py` is a Python package that provides bindings for `image_transport`. It enables efficient publishing and subscribing of images in Python, leveraging various transport plugins (e.g., `raw`, `compressed`). 
The package allows developers to handle image topics more efficiently and with less overhead than using standard ROS2 topics.

## Usage

### Publishing Images

To publish images using `image_transport_py`, you create an `ImageTransport` object and use it to advertise an image topic. The first parameter for `ImageTransport` is the image transport
node's name which needs to be unique in the namespace. 

Steps:

1. Import Necessary Modules:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from image_transport_py import ImageTransport
```

2. Initialize the Node and ImageTransport:

```python
class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.image_transport = ImageTransport("imagetransport_pub", image_transport="raw")
        self.publisher = self.image_transport.advertise('camera/image', 10)

        # read images at 10Hz frequency
        self.timer = self.create_timer(0.1, self.publish_image)
```

3. Publish Images in the Callback:
```python
    def publish_image(self):
        image_msg = .. # Read image from your devices

        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(image_msg)
```

`advertise_camera` can publish `CameraInfo` along with `Image` message.


### Subscribing to Images

To subscribe to images, use `ImageTransport` to create a subscription to the image topic.

Steps:

1. Import Necessary Modules:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from image_transport_py import ImageTransport
```

2. Initialize the Node and ImageTransport:
```python
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_transport = ImageTransport("imagetransport_sub", image_transport="raw")
        self.subscription = self.image_transport.subscribe('camera/image', self.image_callback, 10)
```

3. Handle Incoming Images:
```python
    def image_callback(self, msg):
        # do something with msg (= sensor_msgs.msg.Image type)
```

`subscribe_camera` will add `CameraInfo` along with `Image` message for the callback.


### Transport Selection

By default, `image_transport` uses the `raw` transport. You can specify a different transport by passing `image_transport` parameter to `ImageTransport`. Alternatively,
you can use your own ROS2 parameter file for the imagetransport node



