// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_INI_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_INI_H

#include <string>

namespace camera_calibration_parsers {

bool writeCalibrationIni(const std::string& file_name, const std::string& camera_name,
                         int width, int height,
                         const double* K, const double* D = NULL,
                         const double* R = NULL, const double* P = NULL);

bool readCalibrationIni(const std::string& file_name, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D = NULL, double* R = NULL, double* P = NULL);

bool parseCalibrationIni(const std::string& buffer, std::string& camera_name,
                         int &width, int &height,
                         double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration_parsers

#endif
