// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_YML_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_YML_H

#include <string>

namespace camera_calibration_parsers {

/**
 * \brief Write calibration parameters to a file in YAML format.
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
bool writeCalibrationYml(const std::string& file_name, const std::string& camera_name,
                         int width, int height,
                         const double* K, const double* D = NULL,
                         const double* R = NULL, const double* P = NULL);

/**
 * \brief Read calibration parameters from a YAML file.
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
bool readCalibrationYml(const std::string& file_name, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration_parsers

#endif
