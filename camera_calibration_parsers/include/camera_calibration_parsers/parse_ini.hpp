/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef CAMERA_CALIBRATION_PARSERS__PARSE_INI_HPP_
#define CAMERA_CALIBRATION_PARSERS__PARSE_INI_HPP_

#include <string>

#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_calibration_parsers/visibility_control.hpp"

namespace camera_calibration_parsers
{

using CameraInfo = sensor_msgs::msg::CameraInfo;

/**
 * \brief Write calibration parameters to a file in INI format.
 *
 * \param out Output stream to write to
 * \param camera_name Name of the camera
 * \param cam_info Camera parameters
 */
CAMERA_CALIBRATION_PARSERS_PUBLIC
bool writeCalibrationIni(
  std::ostream & out, const std::string & camera_name,
  const CameraInfo & cam_info);

/**
 * \brief Read calibration parameters from an INI file.
 *
 * \param in Input stream to read from
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
CAMERA_CALIBRATION_PARSERS_PUBLIC
bool readCalibrationIni(
  std::istream & in, std::string & camera_name,
  CameraInfo & cam_info);

/**
 * \brief Write calibration parameters to a file in INI format.
 *
 * \param file_name File to write
 * \param camera_name Name of the camera
 * \param cam_info Camera parameters
 */
CAMERA_CALIBRATION_PARSERS_PUBLIC
bool writeCalibrationIni(
  const std::string & file_name, const std::string & camera_name,
  const CameraInfo & cam_info);

/**
 * \brief Read calibration parameters from an INI file.
 *
 * \param file_name File to read
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
CAMERA_CALIBRATION_PARSERS_PUBLIC
bool readCalibrationIni(
  const std::string & file_name, std::string & camera_name,
  CameraInfo & cam_info);

/**
 * \brief Parse calibration parameters from a string in memory of INI format.
 *
 * \param buffer Calibration string
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
CAMERA_CALIBRATION_PARSERS_PUBLIC
bool parseCalibrationIni(
  const std::string & buffer, std::string & camera_name,
  CameraInfo & cam_info);

}  // namespace camera_calibration_parsers

#endif  // CAMERA_CALIBRATION_PARSERS__PARSE_INI_HPP_
