cmake_minimum_required(VERSION 3.20)

project(camera_info_manager)

# The C++/C standard comes from ament_ros_defaults (cxx_std_20 / c_std_17),
# provided by the ament_cmake_ros_core::ament_ros_defaults INTERFACE target and
# linked into the library's public interface below, propagating to consumers as
# a minimum standard.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)
endif()

find_package(ament_cmake_gen_version_h REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(sensor_msgs REQUIRED)

# add a library
add_library(${PROJECT_NAME} src/camera_info_manager.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_definitions(${PROJECT_NAME} PRIVATE "CAMERA_INFO_MANAGER_BUILDING_DLL")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ament_cmake_ros_core::ament_ros_defaults
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  sensor_msgs::sensor_msgs)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ament_index_cpp::ament_index_cpp
  camera_calibration_parsers::camera_calibration_parsers
  rcpputils::rcpputils)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(ament_cmake_ros_core ament_index_cpp camera_calibration_parsers rclcpp rclcpp_lifecycle rcpputils sensor_msgs)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}-unit_test
    tests/unit_test.cpp
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(${PROJECT_NAME}-unit_test
    ${PROJECT_NAME}
    sensor_msgs::sensor_msgs
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
  )

  # copy the file in the installation folder to test with package://
  install(FILES tests/test_calibration.yaml DESTINATION share/${PROJECT_NAME}/tests)
endif()

ament_package()

ament_generate_version_header(${PROJECT_NAME})
