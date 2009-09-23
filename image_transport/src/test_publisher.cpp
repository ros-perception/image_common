#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv_latest/CvBridge.h>

void subscriberCB(const image_transport::SingleSubscriberPublisher& pub, const std::string& event)
{
  ROS_INFO("%s: %s", event.c_str(), pub.getSubscriberCallerID().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("raw_image", 1,
                                                boost::bind(subscriberCB, _1, "   Connection"),
                                                boost::bind(subscriberCB, _1, "Disconnection"));

  //cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
  cv::WImageBuffer1_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE) );
  sensor_msgs::Image msg;
  sensor_msgs::CvBridge::fromIpltoRosImage(image.Ipl(), msg);
  //msg.encoding = "bgr";
  msg.header.frame_id = "base_link";
  
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
