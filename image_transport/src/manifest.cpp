#include <pluginlib/class_list_macros.h>
#include "image_transport/raw_publisher.h"
#include "image_transport/raw_subscriber.h"

PLUGINLIB_DECLARE_CLASS(image_transport, raw_pub, image_transport::RawPublisher, image_transport::PublisherPlugin)

PLUGINLIB_DECLARE_CLASS(image_transport, raw_sub, image_transport::RawSubscriber, image_transport::SubscriberPlugin)
