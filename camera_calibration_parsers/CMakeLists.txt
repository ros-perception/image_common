cmake_minimum_required(VERSION 3.20)

project(camera_calibration_parsers)

# The C++/C standard comes from ament_ros_defaults (cxx_std_20 / c_std_17),
# provided by the ament_cmake_ros_core::ament_ros_defaults INTERFACE target and
# linked into the library's public interface below, propagating to consumers as
# a minimum standard.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_gen_version_h REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# define the library
add_library(${PROJECT_NAME}
  src/parse.cpp
  src/parse_ini.cpp
  src/parse_yml.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ament_cmake_ros_core::ament_ros_defaults
  sensor_msgs::sensor_msgs)
target_link_libraries(${PROJECT_NAME} PRIVATE
  rclcpp::rclcpp
  yaml-cpp::yaml-cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_compile_definitions(${PROJECT_NAME} PRIVATE "CAMERA_CALIBRATION_PARSERS_BUILDING_DLL")

if(CMAKE_COMPILER_IS_GNUCXX)
  target_link_libraries(${PROJECT_NAME} PUBLIC stdc++fs)
endif()

ament_export_targets(export_${PROJECT_NAME})

# define the exe to convert
add_executable(convert src/convert.cpp)
target_link_libraries(convert PRIVATE
  ${PROJECT_NAME}
  rclcpp::rclcpp)

install(
  TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS convert
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake_ros_core rclcpp sensor_msgs yaml_cpp_vendor)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  ament_add_gtest(${PROJECT_NAME}-parse_ini test/test_parse_ini.cpp)
  if(TARGET ${PROJECT_NAME}-parse_ini)
    target_link_libraries(${PROJECT_NAME}-parse_ini ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-parse_yml test/test_parse_yml.cpp)
  if(TARGET ${PROJECT_NAME}-parse_yml)
    target_link_libraries(${PROJECT_NAME}-parse_yml ${PROJECT_NAME})
  endif()

  # End-to-end test of the `convert` executable, driven from Python.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}-parser test/parser.py
    ENV "CONVERT_EXECUTABLE=$<TARGET_FILE:convert>")
endif()

ament_package()

ament_generate_version_header(${PROJECT_NAME})
