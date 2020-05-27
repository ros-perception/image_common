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
find_package(rcpputils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

# define the library
add_library(${PROJECT_NAME}
  src/parse.cpp
  src/parse_ini.cpp
  src/parse_yml.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "CAMERA_CALIBRATION_PARSERS_BUILDING_DLL")

if(CMAKE_COMPILER_IS_GNUCXX)
  target_link_libraries(${PROJECT_NAME} stdc++fs)
endif()

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rcpputils
  sensor_msgs
  yaml_cpp_vendor
)

# TODO: Reenable Python Wrapper with new serialization technique.
#find_package(PythonLibs REQUIRED)
#if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
#  find_package(Boost REQUIRED COMPONENTS python)
#else()
#  find_package(Boost REQUIRED COMPONENTS python3)
#endif()
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
ament_export_dependencies(rclcpp rcpputils sensor_msgs yaml_cpp_vendor)


if(BUILD_TESTING)
  #TODO(mjcarroll) switch back to this once I can fix copyright check
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  #TODO(mjcarroll) Headers need to be .hpp for this to work properly.
  #ament_cppcheck()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
  find_package(ament_cmake_gtest)

  ament_add_gtest(${PROJECT_NAME}-parse_ini test/test_parse_ini.cpp)
  if(TARGET ${PROJECT_NAME}-parse_ini)
    target_link_libraries(${PROJECT_NAME}-parse_ini ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-parse_yml test/test_parse_yml.cpp)
  if(TARGET ${PROJECT_NAME}-parse_yml)
    target_link_libraries(${PROJECT_NAME}-parse_yml ${PROJECT_NAME})
  endif()
endif()

ament_package()
