// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_INI_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_INI_H

#include <string>

namespace camera_calibration_parsers {

/**
 * \brief Write calibration parameters to a file in INI format.
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
bool writeCalibrationIni(const std::string& file_name, const std::string& camera_name,
                         int width, int height,
                         const double* K, const double* D = NULL,
                         const double* R = NULL, const double* P = NULL);

/**
 * \brief Read calibration parameters from an INI file.
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
bool readCalibrationIni(const std::string& file_name, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D = NULL, double* R = NULL, double* P = NULL);

/**
 * \brief Parse calibration parameters from a string in memory of INI format.
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
bool parseCalibrationIni(const std::string& buffer, std::string& camera_name,
                         int &width, int &height,
                         double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration_parsers

#endif
