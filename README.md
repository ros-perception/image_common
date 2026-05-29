# image_common

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-6.4.7-green.svg)](CHANGELOG.rst)

A collection of ROS 2 packages providing core infrastructure for working with images and cameras in robotic systems. Part of the [ROS Perception](https://github.com/ros-perception) stack.

## Packages

### [image_transport](image_transport/)

The core image transport library. Provides a plugin-based architecture for publishing and subscribing to images in raw or compressed form, transparently switching between transports without changing application code.

- `Publisher` / `CameraPublisher` — publish images with optional camera info
- `Subscriber` / `CameraSubscriber` — subscribe to images with transport hints
- `SubscriberFilter` — integration with `message_filters`
- `republish` node — re-publish images between transports
- `list_transports` — list available transport plugins
- Lifecycle node support and QoS override

### [image_transport_py](image_transport_py/)

Python bindings (pybind11) for `image_transport`, exposing `ImageTransport`, `Publisher`, `Subscriber`, `CameraPublisher`, and `CameraSubscriber` to Python 3 nodes.

### [camera_calibration_parsers](camera_calibration_parsers/)

Read and write `sensor_msgs/CameraInfo` calibration data to/from disk.

- YAML (`.yaml` / `.yml`) and INI (`.ini`) format support
- `convert` command-line tool for format conversion

### [camera_info_manager](camera_info_manager/)

C++ `CameraInfoManager` class for camera drivers that need to load, save, and serve calibration data.

- Loads calibration from `file://`, `package://`, and other URL schemes
- Handles `sensor_msgs/SetCameraInfo` service requests
- Thread-safe; supports lifecycle nodes

### [camera_info_manager_py](camera_info_manager_py/)

Pure Python equivalent of `camera_info_manager` for Python-based camera drivers.

- `CameraInfoManager` and `ZoomCameraInfoManager` classes
- YAML-based calibration storage
- Drop-in counterpart to the C++ package

## Installation

### From binary packages

```bash
sudo apt install ros-$ROS_DISTRO-image-common
```

### From source

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros-perception/image_common.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select image_common
```

## Tutorials

Step-by-step tutorials for writing publishers and subscribers with `image_transport` are available in the companion repository:

**[ros-perception/image_transport_tutorials](https://github.com/ros-perception/image_transport_tutorials/)**

Topics covered include:

- Publishing and subscribing to images with `image_transport`
- Using transport hints to select a specific transport
- Writing a custom transport plugin
- Python usage via `image_transport_py`

## ROS 2 Distro Support

| Distro  | Branch      |
|---------|-------------|
| Rolling | `rolling`   |
| Lyrical | `lyrical`   |
| Kilted  | `kilted`    |
| Jazzy   | `jazzy`     |
| Iron    | `iron`      |
| Humble  | `humble`    |


## Contributing

Contributions are welcome. Please open issues and pull requests on [GitHub](https://github.com/ros-perception/image_common).

## License

BSD 3-Clause. See [LICENSE](LICENSE) for details.
