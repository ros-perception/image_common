// Copyright (c) 2010 Jack O'Quin
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#ifndef _WIN32
#include <unistd.h>
#endif

#include <chrono>
#include <filesystem>
#include <memory>
#include <thread>
#include <cstdlib>
#include <string>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "rcpputils/env.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"

#include "rclcpp/rclcpp.hpp"

///////////////////////////////////////////////////////////////
// global test data
///////////////////////////////////////////////////////////////

class CameraInfoManagerTesting : public ::testing::Test
{
protected:
  void SetUp()
  {
    node = rclcpp::Node::make_shared("camera_info_manager_test");
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    executor_thread = std::thread([this]() {executor->spin();});
  }

  void TearDown()
  {
    executor->cancel();
    if (executor_thread.joinable()) {
      executor_thread.join();
    }
  }

  // Helper to construct CameraInfoManager without deprecated rclcpp::Node* API
  camera_info_manager::CameraInfoManager make_cinfo(
    const std::string & cname = "camera",
    const std::string & url = "",
    const std::string & ns = "")
  {
    return camera_info_manager::CameraInfoManager(
      node->get_node_base_interface(),
      node->get_node_services_interface(),
      node->get_node_logging_interface(),
      cname, url, rclcpp::SystemDefaultsQoS(), ns);
  }

  rclcpp::Node::SharedPtr node;
  std::string g_package_name = "camera_info_manager";
  std::string g_test_name = "test_calibration";
  std::string g_package_filename = "/tests/" + g_test_name + ".yaml";
  std::string g_package_url = "package://" + g_package_name + g_package_filename;
  std::string g_package_name_url = "package://" + g_package_name + "/tests/${NAME}.yaml";
  std::string g_default_url = "file://${ROS_HOME}/camera_info/${NAME}.yaml";
  std::string g_camera_name = "08144361026320a0";
  std::thread executor_thread;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
};

///////////////////////////////////////////////////////////////
// utility functions
///////////////////////////////////////////////////////////////

// compare CameraInfo fields that are saved and loaded for calibration
void compare_calibration(
  const sensor_msgs::msg::CameraInfo & exp,
  const sensor_msgs::msg::CameraInfo & ci)
{
  // check image size
  EXPECT_EQ(exp.width, ci.width);
  EXPECT_EQ(exp.height, ci.height);

  // check distortion coefficients
  EXPECT_EQ(exp.distortion_model, ci.distortion_model);
  EXPECT_EQ(exp.d.size(), ci.d.size());
  for (unsigned i = 0; i < ci.d.size(); ++i) {
    EXPECT_EQ(exp.d[i], ci.d[i]);
  }

  // check camera matrix
  for (unsigned i = 0; i < ci.k.size(); ++i) {
    EXPECT_EQ(exp.k[i], ci.k[i]);
  }

  // check rectification matrix
  for (unsigned i = 0; i < ci.r.size(); ++i) {
    EXPECT_EQ(exp.r[i], ci.r[i]);
  }

  // check projection matrix
  for (unsigned i = 0; i < ci.p.size(); ++i) {
    EXPECT_EQ(exp.p[i], ci.p[i]);
  }
}

// make sure this file does not exist
void delete_file(std::string filename)
{
  std::filesystem::remove(filename);
}

void delete_default_file(void)
{
  std::filesystem::path ros_home;
  std::string ros_home_env = rcpputils::get_env_var("ROS_HOME");
  if (!ros_home_env.empty()) {
    ros_home = ros_home_env;
  } else {
#ifdef _WIN32
    std::string home_env = rcpputils::get_env_var("USERPROFILE");
#else
    std::string home_env = rcpputils::get_env_var("HOME");
#endif
    if (!home_env.empty()) {
      ros_home = std::filesystem::path(home_env) / ".ros";
    }
  }

  if (ros_home.empty()) {
    return;
  }
  std::filesystem::remove(ros_home / "camera_info" / "camera.yaml");
}

void delete_tmp_camera_info_directory(void)
{
  std::filesystem::remove_all(std::filesystem::temp_directory_path() / "camera_info");
}

void make_tmp_camera_info_directory(void)
{
  std::filesystem::create_directories(
    std::filesystem::temp_directory_path() / "camera_info");
}

// These data must match the contents of test_calibration.yaml.
sensor_msgs::msg::CameraInfo expected_calibration(void)
{
  sensor_msgs::msg::CameraInfo ci;

  ci.width = 640u;
  ci.height = 480u;

  // set distortion coefficients
  ci.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  ci.d.resize(5);
  ci.d[0] = -1.04482;
  ci.d[1] = 1.59252;
  ci.d[2] = -0.0196308;
  ci.d[3] = 0.0287906;
  ci.d[4] = 0.0;

  // set camera matrix
  ci.k[0] = 1168.68;
  ci.k[1] = 0.0;
  ci.k[2] = 295.015;
  ci.k[3] = 0.0;
  ci.k[4] = 1169.01;
  ci.k[5] = 252.247;
  ci.k[6] = 0.0;
  ci.k[7] = 0.0;
  ci.k[8] = 1.0;

  // set rectification matrix
  ci.r[0] = 1.0;
  ci.r[1] = 0.0;
  ci.r[2] = 0.0;
  ci.r[3] = 0.0;
  ci.r[4] = 1.0;
  ci.r[5] = 0.0;
  ci.r[6] = 0.0;
  ci.r[7] = 0.0;
  ci.r[8] = 1.0;

  // set projection matrix
  ci.p[0] = ci.k[0];
  ci.p[1] = ci.k[1];
  ci.p[2] = ci.k[2];
  ci.p[3] = 0.0;
  ci.p[4] = ci.k[3];
  ci.p[5] = ci.k[4];
  ci.p[6] = ci.k[5];
  ci.p[7] = 0.0;
  ci.p[8] = ci.k[6];
  ci.p[9] = ci.k[7];
  ci.p[10] = ci.k[8];
  ci.p[11] = 0.0;

  return ci;
}

// issue SetCameraInfo service request
bool set_calibration(
  std::shared_ptr<rclcpp::Node> node,
  const sensor_msgs::msg::CameraInfo & calib)
{
  auto client = node->create_client<sensor_msgs::srv::SetCameraInfo>("set_camera_info");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(
      node->get_logger(), "waiting for service to appear... %s",
      client->get_service_name());
  }

  auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
  request->camera_info = calib;
  auto result_future = client->async_send_request(request);
  // Wait for result; the node is already being spun by executor_thread
  auto status = result_future.wait_for(std::chrono::seconds(5));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "service call timed out");
    client->remove_pending_request(result_future);
    return false;
  }
  return result_future.get()->success;
}

// resolve URL string, result should be as expected
void check_url_substitution(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & url,
  const std::string & exp_url,
  const std::string & camera_name)
{
  camera_info_manager::CameraInfoManager cinfo(
    node->get_node_base_interface(),
    node->get_node_services_interface(),
    node->get_node_logging_interface(),
    camera_name, url, rclcpp::SystemDefaultsQoS());
  std::string sub_url = cinfo.resolveURL(url, camera_name);
  EXPECT_EQ(sub_url, exp_url);
}

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test that valid camera names are accepted
TEST_F(CameraInfoManagerTesting, validNames)
{
  auto cinfo = make_cinfo();

  EXPECT_TRUE(cinfo.setCameraName(std::string("a")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("1")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("_")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("abcdefghijklmnopqrstuvwxyz")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("ABCDEFGHIJKLMNOPQRSTUVWXYZ")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("0123456789")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("0123456789abcdef")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("A1")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("9z")));
  EXPECT_TRUE(cinfo.setCameraName(std::string("08144361026320a0_640x480_mono8")));
}

// Test that invalid camera names are rejected
TEST_F(CameraInfoManagerTesting, invalidNames)
{
  auto cinfo = make_cinfo();

  EXPECT_FALSE(cinfo.setCameraName(std::string("")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("-21")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("C++")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("file:///tmp/url.yaml")));
  EXPECT_FALSE(cinfo.setCameraName(std::string("file://${INVALID}/xxx.yaml")));
}

// Test that valid URLs are accepted
TEST_F(CameraInfoManagerTesting, validURLs)
{
  auto cinfo = make_cinfo();

  EXPECT_TRUE(cinfo.validateURL(std::string("")));
  EXPECT_TRUE(cinfo.validateURL(std::string("file:///")));
  EXPECT_TRUE(cinfo.validateURL(std::string("file:///tmp/url.yaml")));
  EXPECT_TRUE(cinfo.validateURL(std::string("File:///tmp/url.ini")));
  EXPECT_TRUE(cinfo.validateURL(std::string("FILE:///tmp/url.yaml")));
  EXPECT_TRUE(cinfo.validateURL(g_default_url));
  EXPECT_TRUE(cinfo.validateURL(g_package_url));
  EXPECT_TRUE(cinfo.validateURL(std::string("package://no_such_package/calibration.yaml")));
  EXPECT_TRUE(cinfo.validateURL(std::string("packAge://camera_info_manager/x")));
}

// Test that invalid URLs are rejected
TEST_F(CameraInfoManagerTesting, invalidURLs)
{
  auto cinfo = make_cinfo();

  EXPECT_FALSE(cinfo.validateURL(std::string("file://")));
  EXPECT_FALSE(cinfo.validateURL(std::string("flash:///")));
  EXPECT_FALSE(cinfo.validateURL(std::string("html://ros.org/wiki/camera_info_manager")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package://")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package:///")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package://calibration.yaml")));
  EXPECT_FALSE(cinfo.validateURL(std::string("package://camera_info_manager/")));
}

// Test ability to provide uncalibrated CameraInfo
TEST_F(CameraInfoManagerTesting, uncalibrated)
{
  delete_default_file();

  auto cinfo = make_cinfo();
  EXPECT_FALSE(cinfo.isCalibrated());

  sensor_msgs::msg::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::msg::CameraInfo exp;
  compare_calibration(exp, ci);
}

// Test ability to load calibrated CameraInfo
TEST_F(CameraInfoManagerTesting, calibrated)
{
  delete_default_file();

  auto cinfo = make_cinfo();
  EXPECT_FALSE(cinfo.isCalibrated());

  std::string pkgPath =
    ament_index_cpp::get_package_share_path("camera_info_manager").generic_string();
  std::string url("file://" + pkgPath + "/tests/test_calibration.yaml");
  EXPECT_TRUE(cinfo.loadCameraInfo(url));
  EXPECT_TRUE(cinfo.isCalibrated());

  sensor_msgs::msg::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  compare_calibration(exp, ci);
}

// Test ability to load calibrated CameraInfo from package
TEST_F(CameraInfoManagerTesting, fromPackage)
{
  auto cinfo = make_cinfo();

  std::cout << "g_package_url " << g_package_url << std::endl;

  EXPECT_TRUE(cinfo.loadCameraInfo(g_package_url));
  EXPECT_TRUE(cinfo.isCalibrated());

  sensor_msgs::msg::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  compare_calibration(exp, ci);
}

// Test ability to access named calibrated CameraInfo from package
TEST_F(CameraInfoManagerTesting, fromPackageWithName)
{
  auto cinfo = make_cinfo(g_test_name, g_package_name_url);
  EXPECT_TRUE(cinfo.isCalibrated());

  sensor_msgs::msg::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  compare_calibration(exp, ci);
}

// Test load of unresolved "package:" URL files
TEST_F(CameraInfoManagerTesting, unresolvedLoads)
{
  auto cinfo = make_cinfo();

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("package://")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("package:///calibration.yaml")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_THROW(cinfo.loadCameraInfo(std::string("package://no_such_package/calibration.yaml")),
    std::logic_error);
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(
    cinfo.loadCameraInfo(
      std::string("package://camera_info_manager/no_such_file.yaml")));
  EXPECT_FALSE(cinfo.isCalibrated());
}

// Test load of "package:" URL after changing name
TEST_F(CameraInfoManagerTesting, nameChange)
{
  const std::string missing_file("no_such_file");

  // first declare using non-existent camera name
  auto cinfo = make_cinfo(missing_file, g_package_name_url);
  EXPECT_FALSE(cinfo.isCalibrated());

  // set name so it resolves to a test file that does exist
  EXPECT_TRUE(cinfo.setCameraName(g_test_name));
  EXPECT_TRUE(cinfo.isCalibrated());
  sensor_msgs::msg::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  compare_calibration(exp, ci);
}

// Test load of invalid CameraInfo URLs
TEST_F(CameraInfoManagerTesting, invalidLoads)
{
  auto cinfo = make_cinfo();

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("flash:///")));
  EXPECT_FALSE(cinfo.isCalibrated());

  EXPECT_FALSE(cinfo.loadCameraInfo(std::string("html://ros.org/wiki/camera_info_manager")));
  EXPECT_FALSE(cinfo.isCalibrated());

  sensor_msgs::msg::CameraInfo ci(cinfo.getCameraInfo());
  sensor_msgs::msg::CameraInfo exp;
  compare_calibration(exp, ci);
}

// Test ability to set CameraInfo directly
TEST_F(CameraInfoManagerTesting, setCameraInfo)
{
  auto cinfo = make_cinfo();

  // issue calibration service request
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  bool success = cinfo.setCameraInfo(exp);
  EXPECT_TRUE(success);

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success) {
    // check that it worked
    EXPECT_TRUE(cinfo.isCalibrated());
    sensor_msgs::msg::CameraInfo ci = cinfo.getCameraInfo();
    compare_calibration(exp, ci);
  }
}

// Test ability to set calibrated CameraInfo
TEST_F(CameraInfoManagerTesting, setCalibration)
{
  auto cinfo = make_cinfo();

  // issue calibration service request
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  bool success = set_calibration(node, exp);

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success) {
    // check that it worked
    EXPECT_TRUE(cinfo.isCalibrated());
    sensor_msgs::msg::CameraInfo ci = cinfo.getCameraInfo();
    compare_calibration(exp, ci);
  }

#ifdef _WIN32
  std::filesystem::path home_path(rcpputils::get_env_var("USERPROFILE"));
#else
  std::filesystem::path home_path(rcpputils::get_env_var("HOME"));
#endif
  delete_file((home_path / ".ros" / "camera_info" / "camera.yaml").string());
}

// Test ability to save calibrated CameraInfo in default URL
TEST_F(CameraInfoManagerTesting, saveCalibrationDefault)
{
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  bool success;

  // Set ${ROS_HOME} to temp dir, then delete the camera_info
  // directory and everything in it.
  std::string tmp_dir =
    std::filesystem::temp_directory_path().generic_string();
  rcpputils::set_env_var("ROS_HOME", tmp_dir.c_str());
  delete_tmp_camera_info_directory();
  make_tmp_camera_info_directory();

  {
    // create instance to save calibrated data
    auto cinfo = make_cinfo();
    EXPECT_FALSE(cinfo.isCalibrated());

    // issue calibration service request
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success) {
    // create a new instance to load saved calibration
    auto cinfo2 = make_cinfo();
    EXPECT_TRUE(cinfo2.isCalibrated());
    if (cinfo2.isCalibrated()) {
      sensor_msgs::msg::CameraInfo ci(cinfo2.getCameraInfo());
      compare_calibration(exp, ci);
    }
  }
}

// Test ability to save calibrated CameraInfo to default location with
// explicit camera name
TEST_F(CameraInfoManagerTesting, saveCalibrationCameraName)
{
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  bool success;

  // set ${ROS_HOME} to temp dir, delete the calibration file
  std::string tmp_dir =
    std::filesystem::temp_directory_path().generic_string();
  rcpputils::set_env_var("ROS_HOME", tmp_dir.c_str());

  std::string tmpFile = (std::filesystem::temp_directory_path() / "camera_info" /
    (g_camera_name + ".yaml")).generic_string();
  delete_file(tmpFile);
  make_tmp_camera_info_directory();
  {
    // create instance to save calibrated data
    auto cinfo = make_cinfo(g_camera_name);
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success) {
    // create a new instance to load saved calibration
    auto cinfo2 = make_cinfo();
    std::string url = "file://" + tmpFile;
    cinfo2.loadCameraInfo(std::string(url));
    EXPECT_TRUE(cinfo2.isCalibrated());
    if (cinfo2.isCalibrated()) {
      sensor_msgs::msg::CameraInfo ci(cinfo2.getCameraInfo());
      compare_calibration(exp, ci);
    }
  }
  delete_file(tmpFile);
}

// // Test ability to save calibrated CameraInfo in a file
// TEST_F(CameraInfoManagerTesting, saveCalibrationFile)
// {
//   return;

//   ros::NodeHandle node;
//   sensor_msgs::msg::CameraInfo exp(expected_calibration());
//   std::string cname("camera");
//   std::string tmpFile("/tmp/camera_info_manager_calibration_test.yaml");
//   std::string url = "file://" + tmpFile;
//   bool success;

//   // first, delete the file
//   delete_file(tmpFile);

//   {
//     // create instance to save calibrated data
//     camera_info_manager::CameraInfoManager cinfo(node, cname, url);
//     success = set_calibration(node, exp);
//     EXPECT_TRUE(cinfo.isCalibrated());
//   }

//   // only check results if the service succeeded, avoiding confusing
//   // and redundant failure messages
//   if (success) {
//     // create a new instance to load saved calibration
//     camera_info_manager::CameraInfoManager cinfo2(node, cname, url);
//     EXPECT_TRUE(cinfo2.isCalibrated());
//     if (cinfo2.isCalibrated()) {
//       sensor_msgs::msg::CameraInfo ci(cinfo2.getCameraInfo());
//       compare_calibration(exp, ci);
//     }
//   }
// }

// Test ability to save calibrated CameraInfo in a package
// (needs write access).
TEST_F(CameraInfoManagerTesting, saveCalibrationPackage)
{
  sensor_msgs::msg::CameraInfo exp(expected_calibration());
  std::string filename(g_package_filename);
  bool success;

  // first, delete the file
  delete_file(filename);

  {
    // create instance to save calibrated data
    auto cinfo = make_cinfo(g_camera_name, g_package_url);
    success = set_calibration(node, exp);
    EXPECT_TRUE(cinfo.isCalibrated());
  }

  // only check results if the service succeeded, avoiding confusing
  // and redundant failure messages
  if (success) {
    // create a new instance to load saved calibration
    auto cinfo2 = make_cinfo(g_camera_name, g_package_url);
    EXPECT_TRUE(cinfo2.isCalibrated());
    if (cinfo2.isCalibrated()) {
      sensor_msgs::msg::CameraInfo ci(cinfo2.getCameraInfo());
      compare_calibration(exp, ci);
    }
  }
}

TEST_F(CameraInfoManagerTesting, cameraName)
{
  std::string name_url;
  std::string exp_url;

  // resolve a GUID camera name
  name_url = "package://" + g_package_name + "/tests/${NAME}.yaml";
  exp_url = "package://" + g_package_name + "/tests/" + g_camera_name + ".yaml";
  check_url_substitution(node, name_url, exp_url, g_camera_name);

  // substitute camera name "test"
  name_url = "package://" + g_package_name + "/tests/${NAME}_calibration.yaml";
  std::string test_name("test");
  exp_url = "package://" + g_package_name + "/tests/" + test_name + "_calibration.yaml";
  check_url_substitution(node, name_url, exp_url, test_name);

  // with an '_' in the name
  test_name = "camera_1024x768";
  exp_url = "package://" + g_package_name + "/tests/" + test_name + "_calibration.yaml";
  check_url_substitution(node, name_url, exp_url, test_name);

  // substitute empty camera name
  name_url = "package://" + g_package_name + "/tests/${NAME}_calibration.yaml";
  std::string empty_name("");
  exp_url = "package://" + g_package_name + "/tests/" + empty_name + "_calibration.yaml";
  check_url_substitution(node, name_url, exp_url, empty_name);

  // substitute test camera calibration from this package
  check_url_substitution(node, g_package_name_url, g_package_url, g_test_name);
}

TEST_F(CameraInfoManagerTesting, rosHome)
{
  std::string name_url;
  std::string exp_url;

  // resolve ${ROS_HOME} with environment variable undefined
  rcpputils::set_env_var("ROS_HOME", nullptr);
  name_url = "file://${ROS_HOME}/camera_info/test_camera.yaml";
#ifdef _WIN32
  std::string home = rcpputils::get_env_var("USERPROFILE");
#else
  std::string home = rcpputils::get_env_var("HOME");
#endif
  // resolveURL uses generic_string() so forward slashes are always used in URLs
  exp_url = "file://" +
    (std::filesystem::path(home) / ".ros" / "camera_info" / "test_camera.yaml").generic_string();
  check_url_substitution(node, name_url, exp_url, g_camera_name);

  // resolve ${ROS_HOME} with environment variable defined
  rcpputils::set_env_var("ROS_HOME", "/my/ros/home");
  name_url = "file://${ROS_HOME}/camera_info/test_camera.yaml";
  exp_url = "file:///my/ros/home/camera_info/test_camera.yaml";
  check_url_substitution(node, name_url, exp_url, g_camera_name);
}

TEST_F(CameraInfoManagerTesting, unmatchedDollarSigns)
{
  std::string tmp_dir =
    (std::filesystem::temp_directory_path() / "").generic_string();

  // test for "$$" in the URL (NAME should be resolved)
  std::string name_url("file://" + tmp_dir + "$${NAME}.yaml");
  std::string exp_url("file://" + tmp_dir + "$" + g_camera_name + ".yaml");
  check_url_substitution(node, name_url, exp_url, g_camera_name);

  // test for "$" in middle of string
  name_url = "file:///$whatever.yaml";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // test for "$$" in middle of string
  name_url = "file:///something$$whatever.yaml";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // test for "$$" at end of string
  name_url = "file:///$$";
  check_url_substitution(node, name_url, name_url, g_camera_name);
}

TEST_F(CameraInfoManagerTesting, emptyURL)
{
  // test that empty URL is handled correctly

  std::string empty_url("");
  check_url_substitution(node, empty_url, empty_url, g_camera_name);
}

TEST_F(CameraInfoManagerTesting, invalidVariables)
{
  std::string name_url;

  // missing "{...}"
  name_url = "file:///$NAME.yaml";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // invalid substitution variable name
  name_url = "file:///${INVALID}/calibration.yaml";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // truncated substitution variable
  name_url = "file:///${NAME";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // missing substitution variable
  name_url = "file:///${}";
  check_url_substitution(node, name_url, name_url, g_camera_name);

  // no exception thrown for single "$" at end of string
  name_url = "file:///$";
  check_url_substitution(node, name_url, name_url, g_camera_name);
}

// Run all the tests that were declared with TEST_F()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
