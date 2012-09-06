# generate the server
find_package(genmsg REQUIRED)

find_package(catkin REQUIRED sensor_msgs std_msgs)

return()
add_service_files(DIRECTORY srv FILES GetPolledImage.srv)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)


# create some library and exe
add_library(polled_camera src/publication_server.cpp)

add_executable(poller src/poller.cpp)
