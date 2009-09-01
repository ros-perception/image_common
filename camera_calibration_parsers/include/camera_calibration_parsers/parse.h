// license

#ifndef CAMERA_CALIBRATION_PARSERS_PARSE_H
#define CAMERA_CALIBRATION_PARSERS_PARSE_H

#include <string>

namespace camera_calibration_parsers {

// @todo: use stream-based API, so no read/parse distinction
bool writeCalibration(const std::string& file_name, const std::string& camera_name,
                      int width, int height,
                      const double* K, const double* D = NULL,
                      const double* R = NULL, const double* P = NULL);

bool readCalibration(const std::string& file_name, std::string& camera_name,
                     int &width, int &height,
                     double* K, double* D = NULL, double* R = NULL, double* P = NULL);

bool parseCalibration(const std::string& buffer, const std::string& format,
                      std::string& camera_name, int &width, int &height,
                      double* K, double* D = NULL, double* R = NULL, double* P = NULL);

} //namespace camera_calibration_parsers

#endif
