// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_YML_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_YML_H

#include <string>
#include <sensor_msgs/CameraInfo.h>

namespace camera_calibration_parsers {

/**
 * \brief Write calibration parameters to a file in YAML format.
 *
 * \param file_name File to write
 * \param camera_name Name of the camera
 * \param cam_info Camera parameters
 */
bool writeCalibrationYml(const std::string& file_name, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Read calibration parameters from a YAML file.
 *
 * \param file_name File to read
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool readCalibrationYml(const std::string& file_name, std::string& camera_name,
                        sensor_msgs::CameraInfo& cam_info);

} //namespace camera_calibration_parsers

#endif
