cmake_minimum_required(VERSION 3.8)
project(image_transport_tutorial)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# add the resized image message
set(msg_files
  "msg/ResizedImage.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_msgs
)

# add the publisher example
add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rclcpp")

# add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
ament_target_dependencies(my_subscriber
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rclcpp")

# add the plugin example
add_library(resized_plugins src/manifest.cpp src/resized_publisher.cpp src/resized_subscriber.cpp)
ament_target_dependencies(resized_plugins
  "image_transport"
  "OpenCV")

# add the publisher from video example
add_executable(publisher_from_video src/publisher_from_video.cpp)
ament_target_dependencies(publisher_from_video
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rclcpp")

# Install plugin descriptions
pluginlib_export_plugin_description_file(${PROJECT_NAME} resized_plugins.xml)

# Link interface
rosidl_target_interfaces(resized_plugins
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(
  TARGETS my_publisher my_subscriber resized_plugins publisher_from_video
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport pluginlib rosidl_default_runtime rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
