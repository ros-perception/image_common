#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;
  msg = cv_bridge::CvImage(hdr, "bgr8", image).toImageMsg();

  rclcpp::WallRate loop_rate(5);
  while (rclcpp::ok()) {
    pub.publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}

