# image_transport_tutorial

Before starting any of the tutorials below, create a workspace and clone the `image_common` repository so you can inspect and manipulate the code:

```
$ mkdir -p ~/image_transport_ws/src
$ cd ~/image_transport_ws/src
$ git clone --branch ros2 https://github.com/ros-perception/image_common.git
```

Install needed dependencies:

```
$ cd ~/image_transport_ws/
$ source /opt/ros/galactic/setup.bash
$ rosdep install -i --from-path src --rosdistro galactic -y
$ colcon build
```

Make sure to include the correct setup file (in the above example it is for Galactic on Ubuntu and for bash).

## Writing a Simple Image Publisher (C++)
Description: This tutorial shows how to create a publisher node that will continually publish an image.

Tutorial Level: Beginner

Take a look at [my_publisher.cpp](src/my_publisher.cpp).

### The code explained
Now, let's break down the code piece by piece. 
For lines not explained here, review [Writing a Simple Publisher and Subscriber (C++)](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).

```
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
```

These headers will allow us to load an image using OpenCV, convert it to the ROS message format, and publish it.

```
rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
image_transport::ImageTransport it(node);
```

We create an `ImageTransport` instance, initializing it with our node. 
We use methods of `ImageTransport` to create image publishers and subscribers, much as we use methods of `Node` to create generic ROS publishers and subscribers.

```
image_transport::Publisher pub = it.advertise("camera/image", 1);
```

Advertise that we are going to be publishing images on the base topic `camera/image`. 
Depending on whether more plugins are built, additional (per-plugin) topics derived from the base topic may also be advertised. 
The second argument is the size of our publishing queue.

`advertise()` returns an `image_transport::Publisher` object, which serves two purposes:
1. It contains a `publish()` method that lets you publish images onto the base topic it was created with
2. When it goes out of scope, it will automatically unadvertise

```
cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
std_msgs::msg::Header hdr;
sensor_msgs::msg::Image::SharedPtr msg;
msg = cv_bridge::CvImage(hdr, "bgr8", image).toImageMsg();
```

We load a user-specified (on the command line) color image from disk using OpenCV, then convert it to the ROS type `sensor_msgs/msg/Image`.

```
rclcpp::WallRate loop_rate(5);
while (rclcpp::ok()) {
  pub.publish(msg);
  rclcpp::spin_some(node);
  loop_rate.sleep();
}
```

We broadcast the image to anyone connected to one of our topics, exactly as we would have using an `rclcpp::Publisher`.

### Adding video stream from a webcam
The example above requires a path to an image file to be added as a command line parameter. 
This image will be converted and sent as a message to an image subscriber. 
In most cases, however, this is not a very practical example as you are often required to handle streaming data. 
(For example: multiple webcams mounted on a robot record the scene around it and you have to pass the image data to some other node for further analysis). 

The publisher example can be modified quite easily to make it work with a video device supported by `cv::VideoCapture` (in case it is not, you have to handle it accordingly). 
Take a look at [publisher_from_video.cpp](src/publisher_from_video.cpp) to see how a video device can be passed in as a command line argument and used as the image source.

If you have a single device, you do not need to do the whole routine with passing a command line argument. 
In this case, you can hard-code the index/address of the device and directly pass it to the video capturing structure in OpenCV (example: `cv::VideoCapture(0)` if `/dev/video0` is used). 
Multiple checks are also included here to make sure that the publisher does not break if the camera is shut down. 
If the retrieved frame from the video device is not empty, it will then be converted to a ROS message which will be published by the publisher.

## Writing a Simple Image Subscriber (C++)
Description: This tutorial shows how to create a subscriber node that will display an image on the screen. 
By using the `image_transport` subscriber to subscribe to images, any image transport can be used at runtime. 
To learn how to actually use a specific image transport, see the next tutorial.

Tutorial Level: Beginner

Take a look at [my_subscriber.cpp](src/my_subscriber.cpp).

### The code explained
Now, let's break down the code piece by piece.

```
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rclcpp/logging.hpp"
```

These headers will allow us to subscribe to image messages, display images using OpenCV's simple GUI capabilities, and log errors.

```
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
```

This is the callback function that will be called when a new image has arrived on the `camera/image` topic. 
Although the image may have been sent in some arbitrary transport-specific message type, notice that the callback need only handle the normal `sensor_msgs/msg/Image` type. 
All image encoding/decoding is handled automatically for you.

```
try {
  cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::waitKey(10);
} catch (cv_bridge::Exception & e) {
  auto logger = rclcpp::get_logger("my_subscriber");
  RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
```

The body of the callback. 
We convert the ROS image message into an OpenCV image with BGR pixel encoding, then show it in a display window.


```
rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
image_transport::ImageTransport it(node);
```

We create an `ImageTransport` instance, initializing it with our node.

```
image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
```

Subscribe to the `camera/image` base topic. 
The actual ROS topic subscribed to depends on which transport is used. 
In the default case, "raw" transport, the topic is `camera/image` with type `sensor_msgs/msg/Image`. 
ROS will call the `imageCallback` function whenever a new image arrives. 
The 2nd argument is the queue size.

`subscribe()` returns an `image_transport::Subscriber` object that you must hold on to until you want to unsubscribe. 
When the Subscriber object is destructed, it will automatically unsubscribe from the `camera/image` base topic.

In just a few lines of code, we have written a ROS image viewer that can handle images in both raw and a variety of compressed forms.

## Running the Simple Image Publisher and Subscriber with Different Transports
Description: This tutorial discusses running the simple image publisher and subscriber using multiple transports.

Tutorial Level: Beginner

### Running the publisher
In a previous tutorial we made a publisher node called `my_publisher`. 
Now run the node with an image file as the command-line argument:

```
$ ros2 run image_transport_tutorial my_publisher path/to/some/image.jpg
```

To check that your node is running properly, list the topics being published:

```
$ ros2 topic list
```

You should see `/camera/image` in the output. 
You can also get more information about the topic:

```
$ ros2 topic info /camera/image
```

The output should be:

```
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0
```

### Running the subscriber
In the last tutorial, we made a subscriber node called `my_subscriber`. Now run it:

```
$ ros2 run image_transport_tutorial my_subscriber
```

You should see a window pop up with the image you gave to the publisher.

### Finding available transports
`image_transport` searches your ROS installation for transport plugins at runtime and dynamically loads all that are built. 
This affords you great flexibility in adding additional transports, but makes it unclear which are available on your system. 
`image_transport` provides a `list_transports` executable for this purpose:

```
$ ros2 run image_transport list_transports
```

Which should show:

```
Declared transports:
image_transport/raw

Details:
----------
"image_transport/raw"
 - Provided by package: image_transport
 - Publisher: 
      This is the default publisher. It publishes the Image as-is on the base topic.
    
 - Subscriber: 
      This is the default pass-through subscriber for topics of type sensor_msgs/Image.
```

This the expected output for an otherwise new ROS installation after completing the previous tutorials. 
Depending on your setup, you may already have "theora" or other transports available.

### Adding new transports
Our nodes are currently communicating raw `sensor_msgs/msg/Image` messages, so we are not gaining anything over using `rclcpp::Publisher` and `rclcpp::Subscriber`. 
Let's change that by introducing a new transport.

The `compressed_image_transport` package provides plugins for the "compressed" transport, which sends images over the wire in either JPEG- or PNG-compressed form. 
Notice that `compressed_image_transport` is not a dependency of your package; `image_transport` will automatically discover all transport plugins built in your ROS system.

The easiest way to add the "compressed" transport is to install the package:

```
$ sudo apt-get install ros-galactic-compressed-image-transport
```

Or install all the transport plugins at once:

```
$ sudo apt-get install ros-galactic-image-transport-plugins
```

But you can also build from source.

### Changing the transport used
Now let's start up a new subscriber, this one using compressed transport. 
The key is that `image_transport` subscribers check the parameter `_image_transport` for the name of a transport to use in place of "raw". 
Let's set this parameter and start a subscriber node with name "compressed_listener":

```
$ ros2 run image_transport_tutorial my_subscriber --ros-args --remap __name:=compressed_listener -p _image_transport:=compressed
```

You should see an identical image window pop up.

`compressed_listener` is listening to a separate topic carrying JPEG-compressed versions of the same images published on `/camera/image`.

### Changing transport-specific behavior
For a particular transport, we may want to tweak settings such as compression level, bit rate, etc. 
Transport plugins can expose such settings through ROS parameters. 
For example, `/camera/image/compressed` allows you to change the compression format and quality on the fly; see the package documentation for full details.

For now let's adjust the JPEG quality. 
By default, the "compressed" transport uses JPEG compression at 80% quality. 
Let's change it to 15%.
We can use the GUI, `rqt_reconfigure`, to change the quality:

```
$ ros2 run rqt_reconfigure rqt_reconfigure
```

Now pick `/image_publisher` in the drop-down menu and move the `jpeg_quality` slider down to 15%. 
Do you see the compression artifacts in your second view window?

The `rqt_reconfigure` GUI has updated the ROS parameter `/image_publisher/jpeg_quality`. 
You can verify this by running:

```
$ ros2 param get /image_publisher jpeg_quality
```

This should display 15.
