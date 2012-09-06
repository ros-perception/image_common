include_directories(include)

find_package(catkin COMPONENTS sensor_msgs)
include_directories(SYSTEM ${sensor_msgs_INCLUDE_DIRS})

add_library(camera_calibration_parsers src/parse.cpp
                                       src/parse_ini.cpp
                                       src/parse_yml.cpp
)
target_link_libraries(camera_calibration_parsers yml-cpp)

add_executable(convert src/convert.cpp)
target_link_libraries(convert camera_calibration_parsers)
