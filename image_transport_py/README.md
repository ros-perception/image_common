# image_transport_py: Python Bindings for ROS 2 Image Transport

## Introduction

`image_transport_py` is a Python package that provides bindings for `image_transport`. It enables efficient publishing and subscribing of images in Python, leveraging various transport plugins (e.g., `raw`, `compressed`). 
The package allows developers to handle image topics more efficiently and with less overhead than using standard ROS 2 topics.

## Build Status

| ROS2 Distro | Build Status | Package build |
|:---------:|:----:|:----------:|
| Rolling | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__image_transport_py__ubuntu_resolute_amd64)](https://build.ros2.org/job/Rdev__image_transport_py__ubuntu_resolute_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__image_transport_py__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__image_transport_py__ubuntu_resolute_amd64__binary/) |
| Lyrical | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Ldev__image_transport_py__ubuntu_resolute_amd64)](https://build.ros2.org/job/Ldev__image_transport_py__ubuntu_resolute_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__image_transport_py__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__image_transport_py__ubuntu_resolute_amd64__binary/) |
| Kilted | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kdev__image_transport_py__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__image_transport_py__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__image_transport_py__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__image_transport_py__ubuntu_noble_amd64__binary/) |
| Jazzy | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__image_transport_py__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__image_transport_py__ubuntu_noble_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__image_transport_py__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__image_transport_py__ubuntu_noble_amd64__binary/) |
| Humble | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__image_transport_py__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__image_transport_py__ubuntu_jammy_amd64/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__image_transport_py__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__image_transport_py__ubuntu_jammy_amd64__binary/) |

## Usage

The detailed tutorial on `image_transport` and `image_transport_py` can be found at: https://github.com/ros-perception/image_transport_tutorials.

## Classes

### Publisher

A publisher for images.

#### Methods

- `get_topic()`

  Returns the base image topic.

- `get_num_subscribers()`

  Returns the number of subscribers this publisher is connected to.

- `shutdown()`

  Unsubscribe the callback associated with this Publisher.

- `publish(img)`

  Publish an image on the topics associated with this Publisher.

### CameraPublisher

A publisher for images with camera info.

#### Methods

- `get_topic()`

  Returns the base (image) topic of this CameraPublisher.

- `get_num_subscribers()`

  Returns the number of subscribers this camera publisher is connected to.

- `shutdown()`

  Unsubscribe the callback associated with this CameraPublisher.

- `publish(img, info)`

  Publish an image and camera info on the topics associated with this Publisher.

### ImageTransport

An object for image transport operations.

#### Constructor

- `__init__(node_name, image_transport="", launch_params_filepath="")`

  Initialize an ImageTransport object with its node name, `image_transport` and launch params file path. If no `image_transport` specified, the default `raw` plugin will be initialized.

#### Methods

- `advertise(base_topic, queue_size, latch=False)`

  Advertise an image topic.

- `advertise_camera(base_topic, queue_size, latch=False)`

  Advertise an image topic with camera info.

- `subscribe(base_topic, queue_size, callback)`

  Subscribe to an image topic.

- `subscribe_camera(base_topic, queue_size, callback)`

  Subscribe to an image topic with camera info.

### Subscriber

A subscriber for images.

#### Methods

- `get_topic()`

  Returns the base image topic.

- `get_num_publishers()`

  Returns the number of publishers this subscriber is connected to.

- `get_transport()`

  Returns the name of the transport being used.

- `shutdown()`

  Unsubscribe the callback associated with this Subscriber.

### CameraSubscriber

A subscriber for images with camera info.

#### Methods

- `get_topic()`

  Returns the base image topic.

- `get_num_publishers()`

  Returns the number of publishers this subscriber is connected to.

- `get_transport()`

  Returns the name of the transport being used.

- `shutdown()`

  Unsubscribe the callback associated with this CameraSubscriber.
