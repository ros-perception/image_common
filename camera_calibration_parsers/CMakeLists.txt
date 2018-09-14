cmake_minimum_required(VERSION 3.5)

project(camera_calibration_parsers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PythonLibs REQUIRED)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
  find_package(Boost REQUIRED COMPONENTS filesystem python)
else()
  find_package(Boost REQUIRED COMPONENTS filesystem python3)
endif()

include_directories(include)

# define the library
add_library(${PROJECT_NAME}
  src/parse.cpp
  src/parse_ini.cpp
  src/parse_yml.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(${PROJECT_NAME} ${Boost_FILESYSTEM_LIBRARY} ${YAML_CPP_LIBRARIES})

# TODO: Reenable Python Wrapper with new serialization technique.
#add_library(${PROJECT_NAME}_wrapper
#  src/parse_wrapper.cpp)
#ament_target_dependencies(${PROJECT_NAME}_wrapper
#  "rclcpp"
#  "sensor_msgs"
#)
#target_include_directories(${PROJECT_NAME}_wrapper PUBLIC ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME}_wrapper ${PROJECT_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${YAML_CPP_LIBRARIES})

# define the exe to convert
add_executable(convert src/convert.cpp)
target_link_libraries(convert ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
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
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)

if(ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # TODO: Fix tests
  #  add_subdirectory(test)
endif()

ament_package()

