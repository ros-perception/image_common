/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Kentaro Wada.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>

namespace camera_calibration_parsers
{

class PublishCameraInfo
{

public:
  PublishCameraInfo(std::string filename, std::string frame_id, double rate);
  ~PublishCameraInfo();
protected:
  void publishTimerCb(const ros::TimerEvent& event);

  ros::Publisher pub_info_;
  ros::Timer publish_timer_;

  sensor_msgs::CameraInfo info_msg_;
};

PublishCameraInfo::PublishCameraInfo(std::string filename, std::string frame_id, double rate)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string camera_name("camera");
  camera_calibration_parsers::readCalibration(filename, camera_name, info_msg_);
  info_msg_.header.frame_id = frame_id;

  pub_info_ = pnh.advertise<sensor_msgs::CameraInfo>("output", 1);
  publish_timer_ = nh.createTimer(ros::Duration(1.0 / rate), &PublishCameraInfo::publishTimerCb, this);
}

void PublishCameraInfo::publishTimerCb(const ros::TimerEvent& event)
{
  if (pub_info_.getNumSubscribers() <= 0)
  {
    return;
  }

  info_msg_.header.stamp = event.current_real;
  pub_info_.publish(info_msg_);
}

} // namespace camera_calibration_parsers

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_camera_info");

  if (argc < 3)
  {
    ROS_FATAL("Usage: publish_camera_info FILENAME FRAME_ID [PUBLISH_RATE]");
    exit(1);
  }

  std::string filename;
  std::string frame_id;
  filename = std::string(argv[1]);
  frame_id = std::string(argv[2]);

  double rate;
  if (argc == 4)
  {
    // read rate from command line
    rate = atof(argv[3]);
  }
  else
  {
    // read rate from parameter
    ros::NodeHandle pnh("~");
    pnh.param("rate", rate, 30.0);
  }

  camera_calibration_parsers::PublishCameraInfo* publish_info =
      new camera_calibration_parsers::PublishCameraInfo(filename, frame_id, rate);
  ros::spin();
  return 0;
}
