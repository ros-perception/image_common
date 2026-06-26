cmake_minimum_required(VERSION 3.20)

project(image_transport)

# The C++/C standard comes from ament_ros_defaults (cxx_std_20 / c_std_17),
# provided by the ament_cmake_ros_core::ament_ros_defaults INTERFACE target and
# linked into the library's public interface below. Because the public headers
# now use C++20 features (e.g. the defaulted operator<=> on the handle classes),
# the requirement is propagated to consumers as a minimum standard.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_gen_version_h REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tinyxml2 REQUIRED)

# Build image_transport library
add_library(${PROJECT_NAME}
  src/camera_common.cpp
  src/publisher.cpp
  src/publisher_plugin.cpp
  src/subscriber.cpp
  src/subscriber_plugin.cpp
  src/single_subscriber_publisher.cpp
  src/camera_publisher.cpp
  src/camera_subscriber.cpp
  src/image_transport.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ament_cmake_ros_core::ament_ros_defaults
  message_filters::message_filters
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs)
target_link_libraries(${PROJECT_NAME} PRIVATE
  pluginlib::pluginlib
  tinyxml2::tinyxml2)

target_compile_definitions(${PROJECT_NAME} PRIVATE "IMAGE_TRANSPORT_BUILDING_DLL")

# Build image_transport_plugins library (raw)
add_library(${PROJECT_NAME}_plugins
  src/manifest.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME}_plugins ALIAS ${PROJECT_NAME}_plugins)

target_link_libraries(${PROJECT_NAME}_plugins PRIVATE
  ${PROJECT_NAME}
  pluginlib::pluginlib)

# Build list_transports
add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports
  ${PROJECT_NAME}
  pluginlib::pluginlib)

# # Build republish
add_library(republish_node SHARED
  src/republish.cpp
)
target_link_libraries(republish_node
  ${PROJECT_NAME}
  pluginlib::pluginlib
  rclcpp_components::component
  rclcpp::rclcpp
)
rclcpp_components_register_node(republish_node
  PLUGIN "image_transport::Republisher"
  EXECUTABLE republish
)

target_compile_definitions(republish_node PRIVATE "IMAGE_TRANSPORT_BUILDING_DLL")

# Install plugin descriptions
pluginlib_export_plugin_description_file(image_transport default_plugins.xml)

# Install libraries
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

# Install executables
install(
  TARGETS list_transports
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  republish_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install include directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(ament_cmake_ros_core message_filters rclcpp sensor_msgs pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
  )
  ament_lint_auto_find_test_dependencies()

  ament_cppcheck(LANGUAGE "c++")

  find_package(ament_cmake_gtest)
  find_package(rclcpp_lifecycle REQUIRED)

  ament_add_gtest(test_${PROJECT_NAME}-camera_common test/test_camera_common.cpp)
  if(TARGET test_${PROJECT_NAME}-camera_common)
    target_link_libraries(test_${PROJECT_NAME}-camera_common ${PROJECT_NAME})
    target_compile_definitions(test_${PROJECT_NAME}-camera_common PRIVATE
      DEFAULT_PLUGINS_XML="${CMAKE_CURRENT_SOURCE_DIR}/default_plugins.xml"
      TEST_PLUGIN_MANIFEST_XML="${CMAKE_CURRENT_SOURCE_DIR}/test/test_plugin_manifest.xml")
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-publisher test/test_publisher.cpp)
  if(TARGET test_${PROJECT_NAME}-publisher)
    target_link_libraries(test_${PROJECT_NAME}-publisher ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-publisher_lifecycle test/test_publisher_lifecycle.cpp)
  if(TARGET test_${PROJECT_NAME}-publisher_lifecycle)
    target_link_libraries(test_${PROJECT_NAME}-publisher_lifecycle ${PROJECT_NAME} rclcpp_lifecycle::rclcpp_lifecycle)
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-subscriber test/test_subscriber.cpp)
  if(TARGET test_${PROJECT_NAME}-subscriber)
    target_link_libraries(test_${PROJECT_NAME}-subscriber ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-subscriber_lifecycle test/test_subscriber_lifecycle.cpp)
  if(TARGET test_${PROJECT_NAME}-subscriber_lifecycle)
    target_link_libraries(test_${PROJECT_NAME}-subscriber_lifecycle ${PROJECT_NAME} rclcpp_lifecycle::rclcpp_lifecycle)
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-message_passing test/test_message_passing.cpp)
  if(TARGET test_${PROJECT_NAME}-message_passing)
    target_link_libraries(test_${PROJECT_NAME}-message_passing ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-message_passing_lifecycle test/test_message_passing_lifecycle.cpp)
  if(TARGET test_${PROJECT_NAME}-message_passing_lifecycle)
    target_link_libraries(test_${PROJECT_NAME}-message_passing_lifecycle ${PROJECT_NAME} rclcpp_lifecycle::rclcpp_lifecycle)
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-remapping test/test_remapping.cpp)
  if(TARGET test_${PROJECT_NAME}-remapping)
    target_link_libraries(test_${PROJECT_NAME}-remapping ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-remapping_lifecycle test/test_remapping_lifecycle.cpp)
  if(TARGET test_${PROJECT_NAME}-remapping_lifecycle)
    target_link_libraries(test_${PROJECT_NAME}-remapping_lifecycle ${PROJECT_NAME} rclcpp_lifecycle::rclcpp_lifecycle)
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-qos_override test/test_qos_override.cpp)
  if(TARGET test_${PROJECT_NAME}-qos_override)
    target_link_libraries(test_${PROJECT_NAME}-qos_override ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-qos_override_lifecycle test/test_qos_override_lifecycle.cpp)
  if(TARGET test_${PROJECT_NAME}-qos_override_lifecycle)
    target_link_libraries(test_${PROJECT_NAME}-qos_override_lifecycle ${PROJECT_NAME} rclcpp_lifecycle::rclcpp_lifecycle)
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-single_subscriber_publisher test/test_single_subscriber_publisher.cpp)
  if(TARGET test_${PROJECT_NAME}-single_subscriber_publisher)
    target_link_libraries(test_${PROJECT_NAME}-single_subscriber_publisher ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_${PROJECT_NAME}-single_subscriber_publisher_lifecycle test/test_single_subscriber_publisher_lifecycle.cpp)
  if(TARGET test_${PROJECT_NAME}-single_subscriber_publisher_lifecycle)
    target_link_libraries(test_${PROJECT_NAME}-single_subscriber_publisher_lifecycle ${PROJECT_NAME} rclcpp_lifecycle::rclcpp_lifecycle)
  endif()
endif()

ament_package()

ament_generate_version_header(${PROJECT_NAME})
