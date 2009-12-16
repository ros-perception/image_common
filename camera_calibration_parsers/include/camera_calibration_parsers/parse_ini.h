// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_INI_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_INI_H

#include <string>
#include <sensor_msgs/CameraInfo.h>

namespace camera_calibration_parsers {

/**
 * \brief Write calibration parameters to a file in INI format.
 *
 * \param out Output stream to write to
 * \param camera_name Name of the camera
 * \param cam_info Camera parameters
 */
bool writeCalibrationIni(std::ostream& out, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Read calibration parameters from an INI file.
 *
 * \param in Input stream to read from
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool readCalibrationIni(std::istream& in, std::string& camera_name, sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Write calibration parameters to a file in INI format.
 *
 * \param file_name File to write
 * \param camera_name Name of the camera
 * \param cam_info Camera parameters
 */
bool writeCalibrationIni(const std::string& file_name, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Read calibration parameters from an INI file.
 *
 * \param file_name File to read
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool readCalibrationIni(const std::string& file_name, std::string& camera_name,
                        sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Parse calibration parameters from a string in memory of INI format.
 *
 * \param buffer Calibration string
 * \param[out] camera_name Name of the camera
 * \param[out] cam_info Camera parameters
 */
bool parseCalibrationIni(const std::string& buffer, std::string& camera_name,
                         sensor_msgs::CameraInfo& cam_info);

//bool readCalibrationIni

} //namespace camera_calibration_parsers

#endif
