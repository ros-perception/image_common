// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_H

#include <string>
#include <sensor_msgs/CameraInfo.h>

/// @todo: use stream-based API, so no read/parse distinction
namespace camera_calibration_parsers {

/**
 * \brief Write calibration parameters to a file.
 *
 * The file name extension (.yml, .yaml, or .ini) determines the output format.
 *
 * \param file_name File to write
 * \param camera_name Name of the camera
 * \param cam_info Camera parameters
 */
bool writeCalibration(const std::string& file_name, const std::string& camera_name,
                      const sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Read calibration parameters from a file.
 *
 * The file may be YAML or INI format.
 *
 * \param file_name File to read
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool readCalibration(const std::string& file_name, std::string& camera_name,
                     sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Parse calibration parameters from a string in memory.
 *
 * \param buffer Calibration string
 * \param format Format of calibration string, "yml" or "ini"
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool parseCalibration(const std::string& buffer, const std::string& format,
                      std::string& camera_name, sensor_msgs::CameraInfo& cam_info);

} //namespace camera_calibration_parsers

#endif
