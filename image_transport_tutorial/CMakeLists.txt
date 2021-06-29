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
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# add the resized image message
set(msg_files
   "msg/ResizedImage.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES sensor_msgs
)

# Build image_transport_plugins library (raw)
add_library(${PROJECT_NAME}_plugins SHARED
  src/manifest.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})

# add the publisher example
add_executable(my_publisher src/my_publisher.cpp)
ament_target_dependencies(my_publisher
  "cv_bridge"
  "image_transport")
target_link_libraries(my_publisher ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})

# # add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
ament_target_dependencies(my_subscriber
  "cv_bridge"
  "image_transport")
target_link_libraries(my_subscriber ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})

# # Install plugin descriptions
pluginlib_export_plugin_description_file(${PROJECT_NAME} resized_plugins.xml)

# add the plugin example
add_library(resized_publisher src/manifest.cpp src/resized_publisher.cpp src/resized_subscriber.cpp)
target_link_libraries(resized_publisher ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})

### link ###
rosidl_target_interfaces(my_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(my_subscriber
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(resized_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install libraries
install(
  TARGETS ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS my_publisher my_subscriber resized_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugins)

ament_export_dependencies(rclcpp std_msgs sensor_msgs cvbridge image_transport pluginlib message_filters rosidl_default_runtime)

ament_package()