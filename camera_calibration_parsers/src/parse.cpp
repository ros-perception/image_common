#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"

#include <boost/algorithm/string/predicate.hpp>

namespace camera_calibration_parsers {

bool writeCalibration(const std::string& file_name, const std::string& camera_name,
                     int width, int height,
                     const double* K, const double* D,
                     const double* R, const double* P)
{
  if (boost::iends_with(file_name, ".ini"))
    return writeCalibrationIni(file_name, camera_name, width, height, K, D, R, P);
  if (boost::iends_with(file_name, ".yml") || boost::iends_with(file_name, ".yaml"))
    return writeCalibrationYml(file_name, camera_name, width, height, K, D, R, P);

  return false;
}

bool readCalibration(const std::string& file_name, std::string& camera_name,
                    int &width, int &height,
                    double* K, double* D, double* R, double* P)
{
  if (boost::iends_with(file_name, ".ini"))
    return readCalibrationIni(file_name, camera_name, width, height, K, D, R, P);
  if (boost::iends_with(file_name, ".yml") || boost::iends_with(file_name, ".yaml"))
    return readCalibrationYml(file_name, camera_name, width, height, K, D, R, P);

  return false;
}

bool parseCalibration(const std::string& buffer, const std::string& format,
                      std::string& camera_name, int &width, int &height,
                      double* K, double* D, double* R, double* P)
{
  if (format != "ini")
    return false;

  return parseCalibrationIni(buffer, camera_name, width, height, K, D, R, P);
}

} //namespace camera_calibration_parsers
