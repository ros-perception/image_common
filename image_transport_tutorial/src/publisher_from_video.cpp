// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

int main(int argc, char ** argv)
{
  // Check if video source has been passed as a parameter
  if (argv[1] == NULL) {return 1;}

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  // Convert the command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;

  // Check if it is indeed a number
  if (!(video_sourceCmd >> video_source)) {return 1;}

  cv::VideoCapture cap(video_source);
  // Check if video device can be opened with the given index
  if (!cap.isOpened()) {return 1;}
  cv::Mat frame;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;

  rclcpp::WallRate loop_rate(5);
  while (rclcpp::ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if (!frame.empty()) {
      msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}
