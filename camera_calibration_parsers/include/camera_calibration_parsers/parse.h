// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_H

#include <string>
#include <sensor_msgs/CameraInfo.h>

/// @todo: use stream-based API, so no read/parse distinction
namespace camera_calibration_parsers {

bool writeCalibration(const std::string& file_name, const std::string& camera_name,
                      const sensor_msgs::CameraInfo& cam_info);

bool readCalibration(const std::string& file_name, std::string& camera_name,
                     sensor_msgs::CameraInfo& cam_info);

bool parseCalibration(const std::string& buffer, const std::string& format,
                      std::string& camera_name, sensor_msgs::CameraInfo& cam_info);

/**
 * \brief Write calibration parameters to a file.
 *
 * The file name extension (.yml, .yaml, or .ini) determines the output format.
 *
 * \param file_name File to write
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
bool writeCalibration(const std::string& file_name, const std::string& camera_name,
                      int width, int height,
                      const double* K, const double* D = NULL,
                      const double* R = NULL, const double* P = NULL);

/**
 * \brief Read calibration parameters from a file.
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
bool readCalibration(const std::string& file_name, std::string& camera_name,
                     int &width, int &height,
                     double* K, double* D = NULL, double* R = NULL, double* P = NULL);

/**
 * \brief Parse calibration parameters from a string in memory.
 *
 * D, R, P may each be NULL, in which case they are not written to.
 *
 * \param buffer Calibration string
 * \param format Format of calibration string, "yml" or "ini"
 * \param[out] camera_name Name of the camera
 * \param[out] width Horizontal image resolution, in pixels
 * \param[out] height Vertical image resolution, in pixels
 * \param[out] K Camera matrix, a 3x3 row-major array
 * \param[out] D Distortion parameters, 5x1 array
 * \param[out] R Rectification matrix, 3x3 row-major array
 * \param[out] P Projection matrix, 3x4 row-major array
 *
 * @todo Support parsing YML
 */
bool parseCalibration(const std::string& buffer, const std::string& format,
                      std::string& camera_name, int &width, int &height,
                      double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration_parsers

#endif
