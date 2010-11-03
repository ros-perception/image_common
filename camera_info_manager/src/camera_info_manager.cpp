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

#include <string>
#include <locale>
#include <ros/ros.h>
#include <camera_calibration_parsers/parse.h>

#include "camera_info_manager/camera_info_manager.h"

using namespace camera_calibration_parsers;

/** @file

    @brief CameraInfo Manager implementation

    Provides CameraInfo, handles the SetCameraInfo service requests,
    saves and restores sensor_msgs/CameraInfo data.

    @author Jack O'Quin
 */


/** Constructor
 *
 * @param nh node handle, normally for the driver's streaming name
 *           space ("camera").  The service name is relative to this
 *           handle.  Nodes supporting multiple cameras may use
 *           subordinate names, like "left/camera" and "right/camera".
 * @param cname default camera name
 * @param url default Uniform Resource Locator for loading and saving data.
 */
CameraInfoManager::CameraInfoManager(ros::NodeHandle nh,
                                     const std::string &cname,
                                     const std::string &url)
{
  nh_ = nh;                             // save copy of node handle
  camera_name_ = cname;

  // Set the URL and load camera calibration data (if any).
  loadCameraInfo(url);

  // register callback for camera calibration service request
  info_service_ = nh_.advertiseService("set_camera_info",
                                       &CameraInfoManager::setCameraInfo, this);
}

/** load CameraInfo calibration data (if any)
 *
 * @pre mutex_ unlocked
 *
 * @param url a copy of the Uniform Resource Locator
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 *
 * sets cam_info_, if successful
 */
bool CameraInfoManager::loadCalibration(const std::string &url,
                                        const std::string &cname)
{
  bool success = false;                 // return value

  url_type_t url_type = parseURL(url);

  if (url_type != URL_empty)
    {
      ROS_INFO_STREAM("camera calibration URL: " << url);
    }

  switch (url_type)
    {
    case URL_empty:
      {
        ROS_DEBUG("no camera calibration source");
        break;
      }
    case URL_file:
      {
        success = loadCalibrationFile(url.substr(7), cname);
        break;
      }
    case URL_flash:
      {
        ROS_WARN("[CameraInfoManager] reading from flash not implemented yet");
        break;
      }
    case URL_package:
      {
        ROS_WARN("[CameraInfoManager] reading from ROS package not implemented yet");
        break;
      }
    default:
      {
        ROS_ERROR_STREAM("Invalid camera calibration URL: " << url);
        break;
      }
    }

  return success;
}

/** load CameraInfo calibration data from a file
 *
 * @pre mutex_ unlocked
 *
 * @param filename containing CameraInfo to read
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 *
 * Sets cam_info_, if successful
 */
bool CameraInfoManager::loadCalibrationFile(const std::string &filename,
                                            const std::string &cname)
{
  bool success = false;

  ROS_DEBUG_STREAM("reading camera calibration from " << filename);
  std::string cam_name;
  sensor_msgs::CameraInfo cam_info;

  if (readCalibration(filename, cam_name, cam_info))
    {
      if (cname != cam_name)
        {
          ROS_WARN_STREAM("[" << cname << "] does not match name "
                          << cam_name << " in file " << filename);
        }
      success = true;
      {
        // lock only while updating cam_info_
        boost::mutex::scoped_lock lock(mutex_);
        cam_info_ = cam_info;
      }
    }
  else
    {
      ROS_WARN_STREAM("Camera calibration file " << filename << " not found.");
    }

  return success;
}

/** Set a new URL and load its calibration data (if any).
 *
 * @param url new Uniform Resource Locator for CameraInfo.
 * @return true if new URL contains calibration data.
 *
 * cam_info_ updated, if successful.
 */
bool CameraInfoManager::loadCameraInfo(const std::string &url)
{
  std::string cname;
  {
    boost::mutex::scoped_lock lock(mutex_);
    url_ = url;
    cname = camera_name_;
  }

  // load using copies of the parameters, no need to hold the lock
  return loadCalibration(url, cname);
}


/** parse calibration Uniform Resource Locator
 *
 * @param url string to parse
 * @return URL type
 *
 * @note Recognized but unsupported URL types have enum values >= URL_invalid.
 */
CameraInfoManager::url_type_t CameraInfoManager::parseURL(const std::string &url)
{
  if (url == "")
    {
      return URL_empty;
    }
  if (url.substr(0, 8) == "file:///")
    {
      return URL_file;
    }
  if (url.substr(0, 9) == "flash:///")
    {
      return URL_flash;
    }
  if (url.substr(0, 10) == "package://")
    {
      return URL_package;
    }
  return URL_invalid;
}

/** save CameraInfo calibration data
 *
 * @pre mutex_ unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param url is a copy of the URL storage location (if empty, use
 *            "file:///tmp/calibration_<cname>.yaml")
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 */
bool
CameraInfoManager::saveCalibration(const sensor_msgs::CameraInfo &new_info,
                                   const std::string &url,
                                   const std::string &cname)
{
  bool success = false;

  switch (parseURL(url))
    {
    case URL_empty:
      {
        // store using default file name
        std::string filename = "/tmp/calibration_" + cname + ".yaml";
        success = saveCalibrationFile(new_info, filename, cname);
        break;
      }
    case URL_file:
      {
        success = saveCalibrationFile(new_info, url.substr(7), cname);
        break;
      }
    default:
      {
        // invalid URL, save to default location
        ROS_ERROR_STREAM("invalid url: " << url << " (ignored)");
        success = saveCalibration(new_info, std::string(""), cname);
        break;
      }
    }

  return success;
}
  
/** save CameraInfo calibration data to a file
 *
 * @pre mutex_ unlocked
 *
 * @param new_info contains CameraInfo to save
 * @param filename is local file to store data
 * @param cname is a copy of the camera_name_
 * @return true, if successful
 */
bool
CameraInfoManager::saveCalibrationFile(const sensor_msgs::CameraInfo &new_info,
                                       const std::string &filename,
                                       const std::string &cname)
{
  ROS_INFO_STREAM("writing calibration data to " << filename);
  return writeCalibration(filename, cname, new_info);
}

/** Callback for SetCameraInfo request
 *
 * Always updates cam_info_ class variable, even if save fails.
 *
 * @param req SetCameraInfo request message
 * @param rsp SetCameraInfo response message
 * @return true if message handled
 */
bool 
CameraInfoManager::setCameraInfo(sensor_msgs::SetCameraInfo::Request &req,
                                 sensor_msgs::SetCameraInfo::Response &rsp)
{
  // copies of class variables needed for saving calibration
  std::string url_copy;
  std::string cname;
  {
    boost::mutex::scoped_lock lock(mutex_);
    cam_info_ = req.camera_info;
    url_copy = url_;
    cname = camera_name_;
  }

  if (!nh_.ok())
    {
      ROS_ERROR("set_camera_info service called, but driver not running.");
      rsp.status_message = "Camera driver not running.";
      rsp.success = false;
      return false;
    }

  rsp.success = saveCalibration(req.camera_info, url_copy, cname);
  if (!rsp.success)
    rsp.status_message = "Error storing camera calibration.";

  return true;
}

/** Set a new camera name.
 *
 * @param cname new camera name to use for saving calibration data
 * @return true if new name has valid syntax
 *
 * cam_name_ updated, if valid.
 *
 * Valid names contain only alphabetic, numeric, or '_' characters.
 */
bool CameraInfoManager::setCameraName(const std::string &cname)
{
  // the camera name may not be empty
  if (cname.empty())
    return false;

  // validate the camera name characters
  for (unsigned i = 0; i < cname.size(); ++i)
    {
      if (!isalnum(cname[i]) && cname[i] != '_')
        return false;
    }

  // the name is valid, update our private copy
  boost::mutex::scoped_lock lock(mutex_);
  camera_name_ = cname;
  return true;
}

/** Validate URL syntax.
 *
 * @param url Uniform Resource Locator to check
 *
 * @return true if URL syntax is supported by CameraInfoManager
 *              (although the resource need not actually exist)
 */
bool CameraInfoManager::validateURL(const std::string &url)
{
  url_type_t url_type = parseURL(url);
  return (url_type < URL_invalid);
}
