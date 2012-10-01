cmake_minimum_required(VERSION 2.8)
project(camera_calibration_parsers)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED sensor_msgs)
catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               DEPENDS sensor_msgs
)

include_directories(include)

# define the library
find_package(catkin COMPONENTS sensor_msgs rosconsole)
include_directories(SYSTEM ${sensor_msgs_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/parse.cpp
                                       src/parse_ini.cpp
                                       src/parse_yml.cpp
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        COMPONENT main
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# define the exe to convert
add_executable(convert src/convert.cpp)
target_link_libraries(convert ${PROJECT_NAME} ${rosconsole_LIBRARIES})

install(TARGETS convert
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
