// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_INI_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_INI_H

#include <string>
#include <sensor_msgs/CameraInfo.h>

namespace camera_calibration_parsers {

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

/**
 * \brief Write calibration parameters to a file in INI format.
 *
 * \deprecated Use the CameraInfo version instead.
 *
 * \param file_name File to save to
 * \param camera_name Name of the camera
 * \param width Horizontal image resolution, in pixels
 * \param height Vertical image resolution, in pixels
 * \param K Camera matrix, a 3x3 row-major array
 * \param D Distortion parameters, 5x1 array. If NULL, assumes zero distortion.
 * \param R Rectification matrix, 3x3 row-major array. If NULL, assumes the
 * identity matrix.
 * \param P Projection matrix, 3x4 row-major array. If NULL, uses K with an appended
 * column of zeros.
 */
ROSCPP_DEPRECATED
bool writeCalibrationIni(const std::string& file_name, const std::string& camera_name,
                         int width, int height,
                         const double* K, const double* D = NULL,
                         const double* R = NULL, const double* P = NULL);

/**
 * \brief Read calibration parameters from an INI file.
 *
 * \deprecated Use the CameraInfo version instead.
 *
 * D, R, P may each be NULL, in which case they are not written to.
 *
 * \param file_name File to read
 * \param[out] camera_name Name of the camera
 * \param[out] width Horizontal image resolution, in pixels
 * \param[out] height Vertical image resolution, in pixels
 * \param[out] K Camera matrix, a 3x3 row-major array
 * \param[out] D Distortion parameters, 5x1 array
 * \param[out] R Rectification matrix, 3x3 row-major array
 * \param[out] P Projection matrix, 3x4 row-major array
 */
ROSCPP_DEPRECATED
bool readCalibrationIni(const std::string& file_name, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D = NULL, double* R = NULL, double* P = NULL);

/**
 * \brief Parse calibration parameters from a string in memory of INI format.
 *
 * \deprecated Use the CameraInfo version instead.
 *
 * D, R, P may each be NULL, in which case they are not written to.
 *
 * \param buffer Calibration string
 * \param[out] camera_name Name of the camera
 * \param[out] width Horizontal image resolution, in pixels
 * \param[out] height Vertical image resolution, in pixels
 * \param[out] K Camera matrix, a 3x3 row-major array
 * \param[out] D Distortion parameters, 5x1 array
 * \param[out] R Rectification matrix, 3x3 row-major array
 * \param[out] P Projection matrix, 3x4 row-major array
 */
ROSCPP_DEPRECATED
bool parseCalibrationIni(const std::string& buffer, std::string& camera_name,
                         int &width, int &height,
                         double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration_parsers

#endif
