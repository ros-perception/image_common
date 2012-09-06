project(camera_calibration_parsers)
catkin_project(${PROJECT_NAME} INCLUDE_DIRS include)

include_directories(include)

# define the library
find_package(ROS COMPONENTS sensor_msgs rosconsole)
include_directories(SYSTEM ${sensor_msgs_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/parse.cpp
                                       src/parse_ini.cpp
                                       src/parse_yml.cpp
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
        COMPONENT main
)

# define the exe to convert
add_executable(convert src/convert.cpp)
target_link_libraries(convert ${PROJECT_NAME} ${rosconsole_LIBRARIES})

install(TARGETS convert
        DESTINATION share/${PROJECT_NAME}/bin
)
