/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010-2012 Jack O'Quin
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

#include "camera_info_manager/camera_info_manager.hpp"

#include <algorithm>
#include <cstdlib>
#include <locale>
#include <memory>
#include <string>

#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/get_env.hpp"
#include "camera_calibration_parsers/parse.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"


/** @file

    @brief CameraInfo Manager implementation

    Provides CameraInfo, handles the SetCameraInfo service requests,
    saves and restores sensor_msgs/CameraInfo data.

    @author Jack O'Quin
 */

namespace camera_info_manager
{

using camera_calibration_parsers::readCalibration;
using camera_calibration_parsers::writeCalibration;

/** URL to use when no other is defined. */
const std::string
  default_camera_info_url = "file://${ROS_HOME}/camera_info/${NAME}.yaml";

/** Constructor
 *
 * @param node node, normally for the driver's streaming name
 *           space ("camera").  The service name is relative to this
 *           handle.  Nodes supporting multiple cameras may use
 *           subordinate names, like "left/camera" and "right/camera".
 * @param cname default camera name
 * @param url default Uniform Resource Locator for loading and saving data.
 */
CameraInfoManager::CameraInfoManager(
  rclcpp::Node * node,
  const std::string & cname,
  const std::string & url)
: logger_(node->get_logger()),
  camera_name_(cname),
  url_(url),
  loaded_cam_info_(false)
{
  // register callback for camera calibration service request
  info_service_ = node->create_service<SetCameraInfo>(
    "set_camera_info",
    std::bind(
      &CameraInfoManager::setCameraInfoService, this, std::placeholders::_1,
      std::placeholders::_2));
}

/** Get the current CameraInfo data.
 *
 * If CameraInfo has not yet been loaded, an attempt must be made
 * here.  To avoid that, ensure that loadCameraInfo() ran previously.
 * If the load is attempted but fails, an empty CameraInfo will be
 * supplied.
 *
 * The matrices are all zeros if no calibration is available. The
 * image pipeline handles that as uncalibrated data.
 *
 * @warning The caller @em must fill in the message Header of the
 *          CameraInfo returned.  The time stamp and frame_id should
 *          normally be the same as the corresponding Image message
 *          Header fields.
 */
CameraInfo CameraInfoManager::getCameraInfo(void)
{
  while (rclcpp::ok()) {
    std::string cname;
    std::string url;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (loaded_cam_info_) {
        return cam_info_;               // all done
      }

      // load being attempted now
      loaded_cam_info_ = true;

      // copy the name and URL strings
      url = url_;
      cname = camera_name_;
    }  // release the lock

    // attempt load without the lock, it is not recursive
    loadCalibration(url, cname);
  }

  return CameraInfo();
}

/** Get file name corresponding to a @c package: URL.
 *
 * @param url a copy of the Uniform Resource Locator
 * @return file name if package found, "" otherwise
 */
std::string CameraInfoManager::getPackageFileName(const std::string & url)
{
  RCLCPP_DEBUG(logger_, "camera calibration url: %s", url.c_str());

  // Scan URL from after "package://" until next '/' and extract
  // package name.  The parseURL() already checked that it's present.
  size_t prefix_len = std::string("package://").length();
  size_t rest = url.find('/', prefix_len);
  std::string package(url.substr(prefix_len, rest - prefix_len));

  // Look up the ROS package path name.
  std::string pkgPath = ament_index_cpp::get_package_share_directory(package);
  if (pkgPath.empty()) {                // package not found?
    RCLCPP_WARN(logger_, "unknown package: %s (ignored)", package.c_str());
    return pkgPath;
  } else {
    // Construct file name from package location and remainder of URL.
    return pkgPath + url.substr(rest);
  }
}

/** Is the current CameraInfo calibrated?
 *
 * If CameraInfo has not yet been loaded, an attempt must be made
 * here.  To avoid that, ensure that loadCameraInfo() ran previously.
 * If the load failed, CameraInfo will be empty and this predicate
 * will return false.
 *
 * @return true if the current CameraInfo is calibrated.
 */
bool CameraInfoManager::isCalibrated(void)
{
  while (true) {
    std::string cname;
    std::string url;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (loaded_cam_info_) {
        return cam_info_.k[0] != 0.0;
      }

      // load being attempted now
      loaded_cam_info_ = true;

      // copy the name and URL strings
      url = url_;
      cname = camera_name_;
    }   // release the lock

    // attempt load without the lock, it is not recursive
    loadCalibration(url, cname);
  }
}

/** Load CameraInfo calibration data (if any).
 *
 * @pre mutex_ unlocked
 *
 * @param url a copy of the Uniform Resource Locator
 * @param cname is a copy of the camera_name_
 * @return true if URL contains calibration data.
 *
 * sets cam_info_, if successful
 */
bool CameraInfoManager::loadCalibration(
  const std::string & url,
  const std::string & cname)
{
  bool success = false;                 // return value

  const std::string resURL(resolveURL(url, cname));
  url_type_t url_type = parseURL(resURL);

  if (url_type != URL_empty) {
    RCLCPP_INFO(logger_, "camera calibration URL: %s", resURL.c_str());
  }

  switch (url_type) {
    case URL_empty:
      {
        RCLCPP_INFO(logger_, "using default calibration URL");
        success = loadCalibration(default_camera_info_url, cname);
        break;
      }
    case URL_file:
      {
        success = loadCalibrationFile(resURL.substr(7), cname);
        break;
      }
    case URL_flash:
      {
        RCLCPP_WARN(logger_, "reading from flash not implemented yet");
        break;
      }
    case URL_package:
      {
        std::string filename(getPackageFileName(resURL));
        if (!filename.empty()) {
          success = loadCalibrationFile(filename, cname);
        }
        break;
      }
    default:
      {
        RCLCPP_ERROR(logger_, "Invalid camera calibration URL: %s", resURL.c_str());
        break;
      }
  }

  return success;
}

/** Load CameraInfo calibration data from a file.
 *
 * @pre mutex_ unlocked
 *
 * @param filename containing CameraInfo to read
 * @param cname is a copy of the camera_name_
 * @return true if URL contains calibration data.
 *
 * Sets cam_info_, if successful
 */
bool CameraInfoManager::loadCalibrationFile(
  const std::string & filename,
  const std::string & cname)
{
  bool success = false;

  RCLCPP_DEBUG(logger_, "reading camera calibration from %s", filename.c_str());
  std::string cam_name;
  CameraInfo cam_info;

  if (readCalibration(filename, cam_name, cam_info)) {
    if (cname != cam_name) {
      RCLCPP_WARN(
        logger_,
        "[%s] does not match %s in file %s",
        cname.c_str(), cam_name.c_str(), filename.c_str());
    }
    success = true;
    {
      // lock only while updating cam_info_
      std::lock_guard<std::mutex> lock(mutex_);
      cam_info_ = cam_info;
    }
  } else {
    RCLCPP_WARN(logger_, "Camera calibration file %s not found", filename.c_str());
  }

  return success;
}

/** Set a new URL and load its calibration data (if any).
 *
 * If multiple threads call this method simultaneously with different
 * URLs, there is no guarantee which will prevail.
 *
 * @param url new Uniform Resource Locator for CameraInfo.
 * @return true if new URL contains calibration data.
 *
 * @post @c loaded_cam_info_ true (meaning a load was attempted, even
 *       if it failed); @c cam_info_ updated, if successful.
 */
bool CameraInfoManager::loadCameraInfo(const std::string & url)
{
  std::string cname;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    url_ = url;
    cname = camera_name_;
    loaded_cam_info_ = true;
  }

  // load using copies of the parameters, no need to hold the lock
  return loadCalibration(url, cname);
}

/** Resolve Uniform Resource Locator string.
 *
 * @param url a copy of the Uniform Resource Locator, which may
 *            include <tt>${...}</tt> substitution variables.
 * @param cname is a copy of the camera_name_
 *
 * @return a copy of the URL with any variable information resolved.
 */
std::string CameraInfoManager::resolveURL(
  const std::string & url,
  const std::string & cname)
{
  std::string resolved;
  size_t rest = 0;

  while (true) {
    // find the next '$' in the URL string
    size_t dollar = url.find('$', rest);

    if (dollar >= url.length()) {
      // no more variables left in the URL
      resolved += url.substr(rest);
      break;
    }

    // copy characters up to the next '$'
    resolved += url.substr(rest, dollar - rest);

    if (url.substr(dollar + 1, 1) != "{") {
      // no '{' follows, so keep the '$'
      resolved += "$";
    } else if (url.substr(dollar + 1, 6) == "{NAME}") {
      // substitute camera name
      resolved += cname;
      dollar += 6;
    } else if (url.substr(dollar + 1, 10) == "{ROS_HOME}") {
      // substitute $ROS_HOME
      std::string ros_home;
      std::string ros_home_env = rcpputils::get_env_var("ROS_HOME");
      std::string home_env = rcpputils::get_env_var("HOME");
      if (!ros_home_env.empty()) {
        // use environment variable
        ros_home = ros_home_env;
      } else if (!home_env.empty()) {
        // use "$HOME/.ros"
        ros_home = home_env;
        ros_home += "/.ros";
      }
      resolved += ros_home;
      dollar += 10;
    } else {
      // not a valid substitution variable
      RCLCPP_ERROR(logger_, "invalid URL substitution (not resolved): %s", url.c_str());
      resolved += "$";                // keep the bogus '$'
    }

    // look for next '$'
    rest = dollar + 1;
  }

  return resolved;
}

/** Parse calibration Uniform Resource Locator.
 *
 * @param url string to parse
 * @return URL type
 *
 * @note Recognized but unsupported URL types have enum values >= URL_invalid.
 */
CameraInfoManager::url_type_t CameraInfoManager::parseURL(const std::string & url)
{
  if (url == "") {
    return URL_empty;
  }

  // Easy C++14 replacement for boost::iequals from :
  // https://stackoverflow.com/a/4119881
  auto iequals = [](const std::string & a, const std::string & b) {
      return std::equal(
        a.begin(), a.end(),
        b.begin(), b.end(),
        [](char a, char b) {
          return tolower(a) == tolower(b);
        });
    };


  if (iequals(url.substr(0, 8), "file:///")) {
    return URL_file;
  }
  if (iequals(url.substr(0, 9), "flash:///")) {
    return URL_flash;
  }
  if (iequals(url.substr(0, 10), "package://")) {
    // look for a '/' following the package name, make sure it is
    // there, the name is not empty, and something follows it
    size_t rest = url.find('/', 10);
    if (rest < url.length() - 1 && rest > 10) {
      return URL_package;
    }
  }
  return URL_invalid;
}

/** Save CameraInfo calibration data.
 *
 * @pre mutex_ unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param url is a copy of the URL storage location (if empty, use
 *            @c file://${ROS_HOME}/camera_info/${NAME}.yaml)
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 */
bool
CameraInfoManager::saveCalibration(
  const CameraInfo & new_info,
  const std::string & url,
  const std::string & cname)
{
  bool success = false;

  const std::string resURL(resolveURL(url, cname));

  switch (parseURL(resURL)) {
    case URL_empty:
      {
        // store using default file name
        success = saveCalibration(new_info, default_camera_info_url, cname);
        break;
      }
    case URL_file:
      {
        success = saveCalibrationFile(new_info, resURL.substr(7), cname);
        break;
      }
    case URL_package:
      {
        std::string filename(getPackageFileName(resURL));
        if (!filename.empty()) {
          success = saveCalibrationFile(new_info, filename, cname);
        }
        break;
      }
    default:
      {
        // invalid URL, save to default location
        RCLCPP_ERROR(logger_, "invalid url: %s (ignored)", resURL.c_str());
        success = saveCalibration(new_info, default_camera_info_url, cname);
        break;
      }
  }

  return success;
}

/** Save CameraInfo calibration data to a file.
 *
 * @pre mutex_ unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param filename is local file to store data
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 */
bool
CameraInfoManager::saveCalibrationFile(
  const CameraInfo & new_info,
  const std::string & filename,
  const std::string & cname)
{
  RCLCPP_INFO(logger_, "writing calibration data to %s", filename.c_str());

  rcpputils::fs::path filepath(filename);
  rcpputils::fs::path parent = filepath.parent_path();

  if (!rcpputils::fs::exists(parent)) {
    if (!rcpputils::fs::create_directories(parent)) {
      RCLCPP_ERROR(logger_, "unable to create path directory [%s]", parent.string().c_str());
      return false;
    }
  }

  // Directory exists. Permissions might still be bad.
  // Currently, writeCalibration() always returns true no matter what
  // (ros-pkg ticket #5010).
  return writeCalibration(filename, cname, new_info);
}

/** Callback for SetCameraInfo request.
 *
 * Always updates cam_info_ class variable, even if save fails.
 *
 * @param req SetCameraInfo request message
 * @param rsp SetCameraInfo response message
 * @return true if message handled
 */
void
CameraInfoManager::setCameraInfoService(
  const std::shared_ptr<SetCameraInfo::Request> req,
  std::shared_ptr<SetCameraInfo::Response> rsp)
{
  // copies of class variables needed for saving calibration
  std::string url_copy;
  std::string cname;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cam_info_ = req->camera_info;
    url_copy = url_;
    cname = camera_name_;
    loaded_cam_info_ = true;
  }

  if (!rclcpp::ok()) {
    RCLCPP_ERROR(logger_, "set_camera_info service called, but driver not running.");
    rsp->status_message = "Camera driver not running.";
    rsp->success = false;
    return;
  }

  rsp->success = saveCalibration(req->camera_info, url_copy, cname);
  if (!rsp->success) {
    rsp->status_message = "Error storing camera calibration.";
  }
}

/** Set a new camera name.
 *
 * @param cname new camera name to use for saving calibration data
 *
 * @return true if new name has valid syntax; valid names contain only
 *              alphabetic, numeric, or '_' characters.
 *
 * @post @c cam_name_ updated, if valid; since it may affect the URL,
 *       @c cam_info_ will be reloaded before being used again.
 */
bool CameraInfoManager::setCameraName(const std::string & cname)
{
  // the camera name may not be empty
  if (cname.empty()) {
    return false;
  }

  // validate the camera name characters
  for (unsigned i = 0; i < cname.size(); ++i) {
    if (!isalnum(cname[i]) && cname[i] != '_') {
      return false;
    }
  }

  // The name is valid, so update our private copy.  Since the new
  // name might cause the existing URL to resolve somewhere else,
  // force @c cam_info_ to be reloaded before being used again.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_name_ = cname;
    loaded_cam_info_ = false;
  }

  return true;
}

/** Set the camera info manually
 *
 * @param camera_info new camera calibration data
 *
 * @return true if new camera info is set
 *
 * @post @c cam_info_ updated, if valid;
 */
bool CameraInfoManager::setCameraInfo(const CameraInfo & camera_info)
{
  std::lock_guard<std::mutex> lock(mutex_);

  cam_info_ = camera_info;
  loaded_cam_info_ = true;

  return true;
}

/** Validate URL syntax.
 *
 * @param url Uniform Resource Locator to check
 *
 * @return true if URL syntax is supported by CameraInfoManager
 *              (although the resource need not actually exist)
 */
bool CameraInfoManager::validateURL(const std::string & url)
{
  std::string cname;  // copy of camera name
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cname = camera_name_;
  }  // release the lock

  url_type_t url_type = parseURL(resolveURL(url, cname));
  return url_type < URL_invalid;
}
}  // namespace camera_info_manager
