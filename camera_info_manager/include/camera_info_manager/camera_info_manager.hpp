// Copyright (c) 2010-2012 Jack O'Quin
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

#ifndef CAMERA_INFO_MANAGER__CAMERA_INFO_MANAGER_HPP_
#define CAMERA_INFO_MANAGER__CAMERA_INFO_MANAGER_HPP_

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "camera_info_manager/visibility_control.h"

/** @file

    @brief CameraInfo Manager interface

    @author Jack O'Quin
 */

namespace camera_info_manager
{

using CameraInfo = sensor_msgs::msg::CameraInfo;
using SetCameraInfo = sensor_msgs::srv::SetCameraInfo;

/**
 * \brief Manages camera calibration data for a ROS 2 camera driver.
 *
 * Provides \c sensor_msgs::msg::CameraInfo, handles the
 * \c sensor_msgs::srv::SetCameraInfo service, and saves/restores calibration
 * data from the filesystem.
 *
 * \par ROS 2 Service
 *
 * - \b set_camera_info (sensor_msgs/SetCameraInfo) — stores calibration
 *   information sent by a calibration tool.
 *
 * \par Camera Name
 *
 * A camera name is set via the constructor or setCameraName().  It is written
 * into saved calibration files and checked on load; a warning is logged when
 * the name does not match.
 *
 * Syntax: any combination of alphanumeric characters and underscores.
 * Case is significant.  For uniqueness, consider using a GUID or a string
 * that encodes make, model and serial number.
 *
 * \par Calibration URL
 *
 * The location for reading and writing calibration data is expressed as a URL.
 * Supported schemes:
 *
 * - \c file:///full/path/to/calibration.yaml
 * - \c package://ros_package_name/calibrations/camera.yaml
 *
 * The \c package: scheme resolves the path relative to the install prefix of
 * the named ROS package.
 *
 * URL variables (resolved in a single pass, not recursively):
 *
 * - \c ${NAME} — the current camera name.
 * - \c ${ROS_HOME} — value of the \c $ROS_HOME environment variable,
 *   or \c ~/.ros if unset.
 *
 * The default URL when none is given is:
 * \code{.none}
 * file://${ROS_HOME}/camera_info/${NAME}.yaml
 * \endcode
 * If that file exists it is loaded automatically.
 *
 * \par Lazy Loading
 *
 * No calibration data are loaded in the constructor.  The first call to
 * getCameraInfo(), isCalibrated(), or an explicit loadCameraInfo() triggers
 * the actual file read.  Call loadCameraInfo() eagerly to surface load errors
 * before the first getCameraInfo() call.
 *
 * \par Namespace
 *
 * The optional \p ns constructor argument overrides the ROS 2 namespace used
 * for the \c set_camera_info service.  The default is the node's private
 * namespace (\c ~/set_camera_info).
 */

class CameraInfoManager
{
public:
  CAMERA_INFO_MANAGER_PUBLIC
  CameraInfoManager(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logger_interface,
    const std::string & cname = "camera", const std::string & url = "",
    rclcpp::QoS custom_qos = rclcpp::SystemDefaultsQoS(),
    const std::string & ns = "");

  /**
   * \brief Return the current CameraInfo, loading calibration data on first call.
   *
   * \return A copy of the current \c sensor_msgs::msg::CameraInfo.
   *         If calibration has not been loaded yet this call triggers the load.
   *         Returns an empty (uncalibrated) CameraInfo on load failure.
   */
  CAMERA_INFO_MANAGER_PUBLIC
  CameraInfo getCameraInfo(void);

  /**
   * \brief Check whether valid calibration data have been loaded.
   *
   * Triggers a lazy load if calibration has not been attempted yet.
   *
   * \return \c true if calibration data are available.
   */
  CAMERA_INFO_MANAGER_PUBLIC
  bool isCalibrated(void);

  /**
   * \brief Load calibration data from a URL, replacing any previously loaded data.
   *
   * \param url  URL of the calibration file to load (see class documentation
   *             for supported URL schemes and variable substitution).
   * \return \c true on success.
   */
  CAMERA_INFO_MANAGER_PUBLIC
  bool loadCameraInfo(const std::string & url);

  /**
   * \brief Resolve URL substitution variables for a given camera name.
   *
   * Substitutes \c ${NAME} and \c ${ROS_HOME} in \p url using \p cname and
   * the environment.
   *
   * \param url    URL string that may contain substitution variables.
   * \param cname  Camera name to substitute for \c ${NAME}.
   * \return The resolved URL string.
   */
  CAMERA_INFO_MANAGER_PUBLIC
  std::string resolveURL(
    const std::string & url,
    const std::string & cname);

  /**
   * \brief Set the camera name used for URL variable substitution.
   *
   * Updates the stored camera name.  If calibration data have already been
   * loaded, this does not automatically reload them.
   *
   * \param cname  New camera name (alphanumeric and underscores only).
   * \return \c true if \p cname is a syntactically valid camera name.
   */
  CAMERA_INFO_MANAGER_PUBLIC
  bool setCameraName(const std::string & cname);

  /**
   * \brief Directly set (and save) camera calibration data.
   *
   * Stores \p camera_info as the current calibration and attempts to write it
   * to the configured URL.
   *
   * \param camera_info  Calibration data to store.
   * \return \c true if the data were saved successfully.
   */
  CAMERA_INFO_MANAGER_PUBLIC
  bool setCameraInfo(const CameraInfo & camera_info);

  /**
   * \brief Check whether a URL string has a supported scheme.
   *
   * Does not attempt to open or read the resource.
   *
   * \param url  URL to validate.
   * \return \c true if the URL scheme is recognised (\c file: or \c package:).
   */
  CAMERA_INFO_MANAGER_PUBLIC
  bool validateURL(const std::string & url);

private:
  // recognized URL types
  enum class url_type_t
  {
    // supported URLs
    Empty = 0,                   ///< empty string
    File,                        ///< file:
    Package,                     ///< package:
    // URLs not supported (ordered >= Invalid)
    Invalid,                     ///< anything ordered >= here is invalid
    Flash,                       ///< flash:
  };

  // private methods
  std::filesystem::path getPackageFileName(const std::string & url);

  bool loadCalibration(
    const std::string & url,
    const std::string & cname);

  bool loadCalibrationFile(
    const std::filesystem::path & filename,
    const std::string & cname);

  url_type_t parseURL(const std::string & url);

  bool saveCalibration(
    const CameraInfo & new_info,
    const std::string & url,
    const std::string & cname);

  bool saveCalibrationFile(
    const CameraInfo & new_info,
    const std::filesystem::path & filename,
    const std::string & cname);

  void setCameraInfoService(
    const std::shared_ptr<SetCameraInfo::Request> req,
    std::shared_ptr<SetCameraInfo::Response> rsp);

  /** @brief mutual exclusion lock for private data
   *
   *  This non-recursive mutex is only held for a short time while
   *  accessing or changing private class variables.  To avoid
   *  deadlocks and contention, it is never held during I/O or while
   *  invoking a callback.  Most private methods operate on copies of
   *  class variables, keeping the mutex hold time short.
   */
  std::mutex mutex_;

  // private data
  rclcpp::Service<SetCameraInfo>::SharedPtr info_service_;     ///< set_camera_info service
  rclcpp::Logger logger_;               ///< logger
  std::string camera_name_;             ///< camera name
  std::string url_;                     ///< URL for calibration data
  std::string namespace_;               ///< ROS namespace set_camera_info service
  CameraInfo cam_info_;    ///< current CameraInfo
  bool loaded_cam_info_;                ///< cam_info_ load attempted
};  // class CameraInfoManager
}  // namespace camera_info_manager

#endif  // CAMERA_INFO_MANAGER__CAMERA_INFO_MANAGER_HPP_
