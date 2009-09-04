#include "image_transport/image_subscriber.h"
#include "image_transport/image_publisher.h"
#include <ros/names.h>
#include <boost/foreach.hpp>

using namespace image_transport;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_transformer", ros::init_options::AnonymousName);
  if (argc < 2) {
    printf("Usage: %s raw in:=/camera/image_compressed out:=/camera/image_decompressed", argv[0]);
    return 0;
  }
  ros::NodeHandle nh;
  std::string out_topic = nh.resolveName("out");
  std::string in_topic  = nh.resolveName("in");

  ImagePublisher image_pub;
  ImagePublisher::TransportTopicMap& topic_map = image_pub.getTopicMap();
  topic_map.clear();
  topic_map[ argv[1] ] = out_topic;
  image_pub.advertise(nh, out_topic, 1);

  ImageSubscriber image_sub;
  // Use ImagePublisher::publish as the subscriber callback
  typedef void (ImagePublisher::*PublishMemFn)(const sensor_msgs::ImageConstPtr&) const;
  PublishMemFn pub_mem_fn = &ImagePublisher::publish;
  image_sub.subscribe(nh, in_topic, 1, boost::bind(pub_mem_fn, &image_pub, _1));

  ros::spin();

  return 0;
}
