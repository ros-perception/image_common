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

/**
 * \brief Write calibration parameters to a file in YAML format.
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
bool writeCalibrationYml(const std::string& file_name, const std::string& camera_name,
                         int width, int height,
                         const double* K, const double* D = NULL,
                         const double* R = NULL, const double* P = NULL);

/**
 * \brief Read calibration parameters from a YAML file.
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
bool readCalibrationYml(const std::string& file_name, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration_parsers

#endif
