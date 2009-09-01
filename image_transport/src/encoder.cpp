#include "image_transport/image_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "encoder", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  {
  // @todo: need to suppress default publisher
  std::string topic = nh.resolveName("image");
  image_transport::ImagePublisher image_pub;
  image_pub.advertise(nh, topic, 1);
  
  ros::spin();
  }
  return 0;
}
