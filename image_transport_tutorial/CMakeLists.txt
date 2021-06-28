cmake_minimum_required(VERSION 3.8)
project(image_transport_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# add the resized image message
set(msg_files
   "msg/ResizedImage.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}_interface
    ${msg_files}
    DEPENDENCIES builtin_interfaces
)

include_directories(include)

add_library(${PROJECT_NAME}
  src/manifest.cpp
  src/my_publisher.cpp
  src/my_subscriber.cpp
  src/resized_publisher.cpp
  src/resized_subscriber.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "IMAGE_TRANSPORT_BUILDING_DLL")

# Build image_transport_plugins library (raw)
add_library(${PROJECT_NAME}_plugins
  src/manifest.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})

# add the publisher example
add_executable(my_publisher src/my_publisher.cpp)
target_link_libraries(my_publisher ${PROJECT_NAME})

# add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber ${PROJECT_NAME})

# Install plugin descriptions
#pluginlib_export_plugin_description_file(image_transport default_plugins.xml)

# add the plugin example
add_library(resized_publisher src/manifest.cpp src/resized_publisher.cpp src/resized_subscriber.cpp)
target_link_libraries(resized_publisher ${PROJECT_NAME})

# Install libraries
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
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
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs sensor_msgs cvbridge image_transport)

ament_package()