#include "camera_calibration_parsers/parse.h"
#include "camera_calibration_parsers/parse_ini.h"
#include "camera_calibration_parsers/parse_yml.h"

#include <boost/algorithm/string/predicate.hpp>

namespace camera_calibration_parsers {

bool writeCalibration(const std::string& file_name, const std::string& camera_name,
                      const sensor_msgs::CameraInfo& cam_info)
{
  if (boost::iends_with(file_name, ".ini"))
    return writeCalibrationIni(file_name, camera_name, cam_info);
  if (boost::iends_with(file_name, ".yml") || boost::iends_with(file_name, ".yaml"))
    return writeCalibrationYml(file_name, camera_name, cam_info);

  return false;
}

bool readCalibration(const std::string& file_name, std::string& camera_name,
                     sensor_msgs::CameraInfo& cam_info)
{
  if (boost::iends_with(file_name, ".ini"))
    return readCalibrationIni(file_name, camera_name, cam_info);
  if (boost::iends_with(file_name, ".yml") || boost::iends_with(file_name, ".yaml"))
    return readCalibrationYml(file_name, camera_name, cam_info);

  return false;
}

bool parseCalibration(const std::string& buffer, const std::string& format,
                      std::string& camera_name, sensor_msgs::CameraInfo& cam_info)
{
  if (format != "ini")
    return false;

  return parseCalibrationIni(buffer, camera_name, cam_info);
}

} //namespace camera_calibration_parsers
