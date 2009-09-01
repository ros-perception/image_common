// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_YML_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_YML_H

#include <string>

namespace camera_calibration_parsers {

bool writeCalibrationYml(const std::string& file_name, const std::string& camera_name,
                         int width, int height,
                         const double* K, const double* D = NULL,
                         const double* R = NULL, const double* P = NULL);

bool readCalibrationYml(const std::string& file_name, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration_parsers

#endif
