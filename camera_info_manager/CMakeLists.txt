cmake_minimum_required(VERSION 3.5)

project(camera_info_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# add a library
add_library(${PROJECT_NAME} src/camera_info_manager.cpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE "CAMERA_INFO_MANAGER_BUILDING_DLL")

ament_target_dependencies(
  ${PROJECT_NAME}
  ament_index_cpp
  camera_calibration_parsers
  rclcpp
  rcpputils
  sensor_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp camera_calibration_parsers rclcpp rcpputils sensor_msgs)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  #TODO(mjcarroll) switch back to this once I can fix copyright check
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()

  #find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  #TODO(mjcarroll) Headers need to be .hpp for this to work properly.
  #ament_cppcheck()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  #TODO(mjcarroll) Enable unit test with rostest
  #add_executable(unit_test tests/unit_test.cpp)
  #target_link_libraries(unit_test ${PROJECT_NAME})
  #add_rostest(tests/unit_test.test DEPENDENCIES unit_test)
endif()

ament_package()
