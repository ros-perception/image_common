cmake_minimum_required(VERSION 2.8)
project(camera_calibration_parsers)

find_package(catkin REQUIRED sensor_msgs rosconsole)

find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs
)

find_package(PkgConfig)

if (APPLE)
  #find yaml cpp for mac os
  find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp)
  find_path(YAML_CPP_H_INCLUDE_DIR yaml-cpp/yaml.h )
  SET(YAML_CPP_INCLUDE_DIR ${YAML_CPP_H_INCLUDE_DIR}/yaml-cpp)
elseif (ANDROID)
    find_package(yaml-cpp)
    add_definitions(-DHAVE_NEW_YAMLCPP)
else()
  pkg_check_modules(YAML_CPP yaml-cpp)
  if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
    add_definitions(-DHAVE_NEW_YAMLCPP)
  endif()
  link_directories(${YAML_CPP_LIBRARY_DIRS})
endif()
include_directories(${YAML_CPP_INCLUDE_DIRS})

# define the library
add_library(${PROJECT_NAME}
  src/parse.cpp
  src/parse_ini.cpp
  src/parse_yml.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${PROJECT_NAME} sensor_msgs_gencpp)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  COMPONENT main
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# define the exe to convert
add_executable(convert src/convert.cpp)
target_link_libraries(convert ${PROJECT_NAME} ${rosconsole_LIBRARIES})

install(
  TARGETS convert
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
