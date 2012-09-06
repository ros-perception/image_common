add_library(image_transport src/publisher.cpp src/subscriber.cpp src/single_subscriber_publisher.cpp src/image_transport.cpp src/camera_common.cpp src/camera_publisher.cpp src/camera_subscriber.cpp src/manifest.cpp)
find_package(Boost)
target_link_libraries(image_transport ${BOOST_LIBRARIES})

add_executable(republish src/republish.cpp)
target_link_libraries(republish image_transport)

add_executable(list_transports src/list_transports.cpp)
target_link_libraries(list_transports image_transport)
