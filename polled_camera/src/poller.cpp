#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include "polled_camera/GetPolledImage.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poller", ros::init_options::AnonymousName);
  if (argc < 2) {
    printf("Usage: %s <Hz> camera:=<namespace> output:=<namespace>\n", argv[0]);
    return 0;
  }
  double hz = boost::lexical_cast<double>(argv[1]);
  
  ros::NodeHandle nh;
  std::string service_name = nh.resolveName("camera") + "/request_image";
  ros::ServiceClient client = nh.serviceClient<polled_camera::GetPolledImage>(service_name);

  polled_camera::GetPolledImage::Request req;
  polled_camera::GetPolledImage::Response rsp;
  req.response_namespace = nh.resolveName("output");

  ros::Rate loop_rate(hz);
  while (nh.ok()) {
    if (client.call(req, rsp))
      std::cout << "Timestamp: " << rsp.stamp << std::endl;
    else {
      ROS_ERROR("Service call failed");
      ros::Duration(0.2).sleep();
      client.waitForExistence();
    }
  }
}
