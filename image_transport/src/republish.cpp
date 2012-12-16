#include "image_transport/image_transport.h"
#include "image_transport/publisher_plugin.h"
#include <pluginlib/class_loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_republisher", ros::init_options::AnonymousName);
  if (argc < 2) {
    printf("Usage: %s in_transport in:=<in_base_topic> [out_transport] out:=<out_base_topic>\n", argv[0]);
    return 0;
  }
  ros::NodeHandle nh;
  std::string in_topic  = nh.resolveName("in");
  std::string in_transport = argv[1];
  std::string out_topic = nh.resolveName("out");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub;
  
  if (argc < 3) {
    // Use all available transports for output
    image_transport::Publisher pub = it.advertise(out_topic, 1);
    
    // Use Publisher::publish as the subscriber callback
    typedef void (image_transport::Publisher::*PublishMemFn)(const sensor_msgs::ImageConstPtr&) const;
    PublishMemFn pub_mem_fn = &image_transport::Publisher::publish;
    sub = it.subscribe(in_topic, 1, boost::bind(pub_mem_fn, &pub, _1), ros::VoidPtr(), in_transport);

    ros::spin();
  }
  else {
    // Use one specific transport for output
    std::string out_transport = argv[2];

    // Load transport plugin
    typedef image_transport::PublisherPlugin Plugin;
    pluginlib::ClassLoader<Plugin> loader("image_transport", "image_transport::PublisherPlugin");
    std::string lookup_name = Plugin::getLookupName(out_transport);
    boost::shared_ptr<Plugin> pub( loader.createInstance(lookup_name) );
    pub->advertise(nh, out_topic, 1, image_transport::SubscriberStatusCallback(),
                   image_transport::SubscriberStatusCallback(), ros::VoidPtr(), false);

    // Use PublisherPlugin::publish as the subscriber callback
    typedef void (Plugin::*PublishMemFn)(const sensor_msgs::ImageConstPtr&) const;
    PublishMemFn pub_mem_fn = &Plugin::publish;
    sub = it.subscribe(in_topic, 1, boost::bind(pub_mem_fn, pub.get(), _1), pub, in_transport);

    ros::spin();
  }

  return 0;
}
