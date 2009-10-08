#include "camera_calibration_parsers/parse_yml.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ctime>
#include <cassert>
#include <cstring>

namespace camera_calibration_parsers {

/// \cond

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";

struct SimpleMatrix
{
  int rows;
  int cols;
  double* data;

  SimpleMatrix(int rows, int cols, double* data)
    : rows(rows), cols(cols), data(data)
  {}
};

YAML::Emitter& operator << (YAML::Emitter& out, const SimpleMatrix& m)
{
  out << YAML::BeginMap;
  out << YAML::Key << "rows" << YAML::Value << m.rows;
  out << YAML::Key << "cols" << YAML::Value << m.cols;
  //out << YAML::Key << "dt"   << YAML::Value << "d"; // OpenCV data type specifier
  out << YAML::Key << "data" << YAML::Value;
  out << YAML::Flow;
  out << YAML::BeginSeq;
  for (int i = 0; i < m.rows*m.cols; ++i)
    out << m.data[i];
  out << YAML::EndSeq;
  out << YAML::EndMap;
  return out;
}

void operator >> (const YAML::Node& node, SimpleMatrix& m)
{
  int rows, cols;
  node["rows"] >> rows;
  assert(rows == m.rows);
  node["cols"] >> cols;
  assert(cols == m.cols);
  const YAML::Node& data = node["data"];
  for (int i = 0; i < rows*cols; ++i)
    data[i] >> m.data[i];
}

/// \endcond

bool writeCalibrationYml(const std::string& file_name, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info)
{
  YAML::Emitter out;
  out << YAML::BeginMap;

#if 0
  // Calibration time
  // FIXME: this breaks yaml-cpp on reading for some reason
  time_t raw_time;
  time( &raw_time );
  out << YAML::Key << "calibration_time";
  out << YAML::Value << asctime(localtime(&raw_time));
#endif

  // Image dimensions
  out << YAML::Key << WIDTH_YML_NAME << YAML::Value << (int)cam_info.width;
  out << YAML::Key << HEIGHT_YML_NAME << YAML::Value << (int)cam_info.height;
  
  // Camera name and intrinsics
  out << YAML::Key << CAM_YML_NAME << YAML::Value << camera_name;
  out << YAML::Key << K_YML_NAME << YAML::Value << SimpleMatrix(3, 3, const_cast<double*>(&cam_info.K[0]));
  out << YAML::Key << D_YML_NAME << YAML::Value << SimpleMatrix(1, 5, const_cast<double*>(&cam_info.D[0]));
  out << YAML::Key << R_YML_NAME << YAML::Value << SimpleMatrix(3, 3, const_cast<double*>(&cam_info.R[0]));
  out << YAML::Key << P_YML_NAME << YAML::Value << SimpleMatrix(3, 4, const_cast<double*>(&cam_info.P[0]));

  out << YAML::EndMap;

  // Write to file
  FILE* file = fopen(file_name.c_str(), "w");
  if (!file)
    return false;
  fprintf(file, "%s\n", out.c_str());
  fclose(file);
  
  return true;
}

bool readCalibrationYml(const std::string& file_name, std::string& camera_name,
                        sensor_msgs::CameraInfo& cam_info)
{
  std::ifstream fin(file_name.c_str());
  YAML::Parser parser(fin);
  if (!parser)
    printf("Parser not OK!\n");
  YAML::Node doc;
  parser.GetNextDocument(doc);

  try {
    doc[CAM_YML_NAME] >> camera_name;
  } catch(...) {
    camera_name = "unknown";
  }
  doc[WIDTH_YML_NAME] >> cam_info.width;
  doc[HEIGHT_YML_NAME] >> cam_info.height;
  
  SimpleMatrix K_(3, 3, &cam_info.K[0]);
  doc[K_YML_NAME] >> K_;
  SimpleMatrix D_(1, 5, &cam_info.D[0]);
  doc[D_YML_NAME] >> D_;
  SimpleMatrix R_(3, 3, &cam_info.R[0]);
  doc[R_YML_NAME] >> R_;
  SimpleMatrix P_(3, 4, &cam_info.P[0]);
  doc[P_YML_NAME] >> P_;
  
  return true;
}

} //namespace camera_calibration_parsers
