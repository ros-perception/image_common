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

class SaveCameraInfo
{

public:
  SaveCameraInfo();
  ~SaveCameraInfo();
protected:
  ros::Subscriber sub_info_;
  void infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);

  std::string prefix_;
  std::string extension_;
  bool once_;
};

SaveCameraInfo::SaveCameraInfo()
{
  ros::NodeHandle pnh("~");
  pnh.param("prefix", prefix_, std::string("camera_info_"));
  pnh.param("extension", extension_, std::string(".yaml"));
  pnh.param("once", once_, false);
  sub_info_ = pnh.subscribe("input", 1, &SaveCameraInfo::infoCb, this);
}

void SaveCameraInfo::infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  std::ostringstream stream;
  stream << prefix_ << ros::Time::now().toNSec() << extension_;
  camera_calibration_parsers::writeCalibration(stream.str(), info_msg->header.frame_id, *info_msg);
  ROS_INFO("Data saved to %s.", stream.str().c_str());

  if (once_)
  {
    sub_info_.shutdown();
    exit(0);
  }
}

} // namespace camera_calibration_parsers

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_camera_info", ros::init_options::AnonymousName);
  camera_calibration_parsers::SaveCameraInfo* save_info = new camera_calibration_parsers::SaveCameraInfo();
  ros::spin();
  return 0;
}
