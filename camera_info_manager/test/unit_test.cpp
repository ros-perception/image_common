/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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

#include <ros/ros.h>
#include <ros/package.h>
#include "camera_info_manager/camera_info_manager.h"
#include <sensor_msgs/distortion_models.h>
#include <string>
#include <gtest/gtest.h>

///////////////////////////////////////////////////////////////
// utility functions
///////////////////////////////////////////////////////////////

// compare CameraInfo fields that are saved and loaded for calibration
void compare_calibration(const sensor_msgs::CameraInfo &exp, const sensor_msgs::CameraInfo &ci)
{
  // check image size
  EXPECT_EQ(exp.width, ci.width);
  EXPECT_EQ(exp.height, ci.height);

  // check distortion coefficients
  EXPECT_EQ(exp.distortion_model, ci.distortion_model);
  EXPECT_EQ(exp.D.size(), ci.D.size());
  for (unsigned i = 0; i < ci.D.size(); ++i)
    {
      EXPECT_EQ(exp.D[i], ci.D[i]);
    }

  // check camera matrix
  for (unsigned i = 0; i < ci.K.size(); ++i)
    {
      EXPECT_EQ(exp.K[i], ci.K[i]);
    }

  // check rectification matrix
  for (unsigned i = 0; i < ci.R.size(); ++i)
    {
      EXPECT_EQ(exp.R[i], ci.R[i]);
    }

  // check projection matrix
  for (unsigned i = 0; i < ci.P.size(); ++i)
    {
      EXPECT_EQ(exp.P[i], ci.P[i]);
    }
}

// These data must match the contents of test_calibration.yaml.
sensor_msgs::CameraInfo expected_calibration(void)
{
  sensor_msgs::CameraInfo ci;

  ci.width = 640u;
  ci.height = 480u;

  // set distortion coefficients
  ci.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  ci.D.resize(5);
  ci.D[0] = -1.04482;
  ci.D[1] = 1.59252;
  ci.D[2] = -0.0196308;
  ci.D[3] = 0.0287906;
  ci.D[4] = 0.0;

  // set camera matrix
  ci.K[0] = 1168.68;
  ci.K[1] = 0.0;
  ci.K[2] = 295.015;
  ci.K[3] = 0.0;
  ci.K[4] = 1169.01;
  ci.K[5] = 252.247;
  ci.K[6] = 0.0;
  ci.K[7] = 0.0;
  ci.K[8] = 1.0;

  // set rectification matrix
  ci.R[0] = 1.0;
  ci.R[1] = 0.0;
  ci.R[2] = 0.0;
  ci.R[3] = 0.0;
  ci.R[4] = 1.0;
  ci.R[5] = 0.0;
  ci.R[6] = 0.0;
  ci.R[7] = 0.0;
  ci.R[8] = 1.0;

  // set projection matrix
  ci.P[0] = ci.K[0];
  ci.P[1] = ci.K[1];
  ci.P[2] = ci.K[2];
  ci.P[3] = 0.0;
  ci.P[4] = ci.K[3];
  ci.P[5] = ci.K[4];
  ci.P[6] = ci.K[5];
  ci.P[7] = 0.0;
  ci.P[8] = ci.K[6];
  ci.P[9] = ci.K[7];
  ci.P[10] = ci.K[8];
  ci.P[11] = 0.0;

  return ci;
}

// issue SetCameraInfo service request
bool set_calibration(ros::NodeHandle node,
                     const sensor_msgs::CameraInfo &calib)
{
  ros::ServiceClient client =
    node.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info");
  sensor_msgs::SetCameraInfo set_camera_info;
  set_camera_info.request.camera_info = calib;
  bool success;
  EXPECT_TRUE((success = client.call(set_camera_info)));
  return success;
}

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test that valid camera names are accepted
TEST(CameraName, validNames)
{
  ros::NodeHandle node;
  CameraInfoManager cinfo(node);

  EXPECT_TRUE(cinfo.setCameraName(std::string("a")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("1")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("_")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("abcdefghijklmnopqrstuvwxyz")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("ABCDEFGHIJKLMNOPQRSTUVWXYZ")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("0123456789")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("0123456789abcdef")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("A1")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("9z")));
}

// Test that invalid camera names are rejected
TEST(CameraName, invalidNames)
{
  ros::NodeHandle node;
  CameraInfoManager cinfo(node);

  EXPECT_FALSE(cinfo.setCameraName(std::string("")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("-21")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("C++")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("file:///tmp/url.yaml")));
}

// Test that valid URLs are accepted
TEST(CameraName, validURLs)
{
  ros::NodeHandle node;
  CameraInfoManager cinfo(node);

  EXPECT_TRUE(cinfo.validateURL(std::string("")));
  EXPECT_TRUE(cinfo.validateURL(std::string("file:///")));
  EXPECT_TRUE(cinfo.validateURL(std::string("file:///tmp/url.yaml")));
  EXPECT_TRUE(cinfo.validateURL(std::string("file:///tmp/url.ini")));
}

// Test that invalid URLs are rejected
TEST(CameraName, invalidURLs)
{
  ros::NodeHandle node;
  CameraInfoManager cinfo(node);

  EXPECT_FALSE(cinfo.validateURL(std::string("flash:///")));
  EXPECT_FALSE(cinfo.validateURL(std::string("html://ros.org/wiki/camera_info_manager")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package://camera_info_manager/test/test_calibration.yaml")));
}

// Test ability to provide uncalibrated CameraInfo
TEST(getInfo, uncalibrated)
{
  ros::NodeHandle node;
  CameraInfoManager cinfo(node);

  EXPECT_FALSE(cinfo.isCalibrated());

  sensor_msgs::CameraInfo ci = cinfo.getCameraInfo();
  sensor_msgs::CameraInfo exp;
  compare_calibration(exp, ci);
}

// Test ability to load calibrated CameraInfo
TEST(getInfo, calibrated)
{
  ros::NodeHandle node;
  CameraInfoManager cinfo(node);

  EXPECT_FALSE(cinfo.isCalibrated());

  std::string pkgPath = ros::package::getPath("camera_info_manager");
  std::string url = "file:///" + pkgPath + "/test/test_calibration.yaml";
  cinfo.loadCameraInfo(url);

  EXPECT_TRUE(cinfo.isCalibrated());

  sensor_msgs::CameraInfo ci = cinfo.getCameraInfo();
  sensor_msgs::CameraInfo exp = expected_calibration();

  compare_calibration(exp, ci);
}

// Test load of invalid CameraInfo URLs
TEST(getInfo, invalidLoads)
{
  ros::NodeHandle node;
  CameraInfoManager cinfo(node);
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("flash:///")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("html://ros.org/wiki/camera_info_manager")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("package://camera_info_manager/test/test_calibration.yaml")));
  EXPECT_FALSE(cinfo.isCalibrated());

  sensor_msgs::CameraInfo ci = cinfo.getCameraInfo();
  sensor_msgs::CameraInfo exp;
  compare_calibration(exp, ci);
}

// Test ability to set calibrated CameraInfo
TEST(setInfo, setCalibration)
{
  ros::NodeHandle node;
  CameraInfoManager cinfo(node);
  EXPECT_FALSE(cinfo.isCalibrated());

  // issue calibration service request
  sensor_msgs::CameraInfo exp = expected_calibration();
  bool success = set_calibration(node, exp);

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // check that it worked
      EXPECT_TRUE(cinfo.isCalibrated());
      sensor_msgs::CameraInfo ci = cinfo.getCameraInfo();
      compare_calibration(exp, ci);
    }
}

// Test ability to save calibrated CameraInfo in /tmp
TEST(setInfo, saveCalibrationTmp)
{
  ros::NodeHandle node;
  sensor_msgs::CameraInfo exp = expected_calibration();
  bool success;

  {
    // create instance to save calibrated data
    CameraInfoManager cinfo(node);
    EXPECT_FALSE(cinfo.isCalibrated());

    // issue calibration service request
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // create a new instance to load saved calibration
      CameraInfoManager cinfo2(node);
      EXPECT_FALSE(cinfo2.isCalibrated());
      cinfo2.loadCameraInfo(std::string("file:///tmp/calibration_camera.yaml"));
      EXPECT_TRUE(cinfo2.isCalibrated());
      sensor_msgs::CameraInfo ci = cinfo2.getCameraInfo();
      compare_calibration(exp, ci);
    }
}

// Test ability to save calibrated CameraInfo in /tmp with camera name
TEST(setInfo, saveCalibrationCameraName)
{
  ros::NodeHandle node;
  sensor_msgs::CameraInfo exp = expected_calibration();
  std::string cname("08144361026320a0");
  std::string url = "file:///tmp/calibration_" + cname + ".yaml";
  bool success;

  {
    // create instance to save calibrated data
    CameraInfoManager cinfo(node, cname);
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // create a new instance to load saved calibration
      CameraInfoManager cinfo2(node);
      cinfo2.loadCameraInfo(std::string(url));
      EXPECT_TRUE(cinfo2.isCalibrated());
      sensor_msgs::CameraInfo ci = cinfo2.getCameraInfo();
      compare_calibration(exp, ci);
    }
}

// Test ability to save calibrated CameraInfo in a file
TEST(setInfo, saveCalibrationFile)
{
  ros::NodeHandle node;
  sensor_msgs::CameraInfo exp = expected_calibration();
  std::string cname("camera");
  std::string url("file:///tmp/camera_info_manager_calibration_test.yaml");
  bool success;

  {
    // create instance to save calibrated data
    CameraInfoManager cinfo(node, cname, url);
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success)
    {
      // create a new instance to load saved calibration
      CameraInfoManager cinfo2(node, cname, url);
      EXPECT_TRUE(cinfo2.isCalibrated());
      sensor_msgs::CameraInfo ci = cinfo2.getCameraInfo();
      compare_calibration(exp, ci);
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_info_manager_unit_test");
  testing::InitGoogleTest(&argc, argv);

  // create asynchronous thread for handling service requests
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
