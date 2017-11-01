#include <pluginlib/class_list_macros.h>
#include <image_transport_tutorial/resized_publisher.h>
#include <image_transport_tutorial/resized_subscriber.h>

PLUGINLIB_EXPORT_CLASS(ResizedPublisher, image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(ResizedSubscriber, image_transport::SubscriberPlugin)
