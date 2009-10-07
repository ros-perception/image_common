#include "camera_calibration_parsers/parse_ini.h"

#include <boost/shared_ptr.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_file_iterator.hpp>
#include <boost/spirit/include/classic_confix.hpp>
#include <boost/spirit/include/classic_loops.hpp>
#include <boost/typeof/typeof.hpp>
#include <cstdio>

namespace camera_calibration_parsers {

/// @todo Move to new spirit
using namespace BOOST_SPIRIT_CLASSIC_NS;

/// \cond
static void printMatrix(FILE* file, const double* M, int rows, int cols, const char* endline = "\n")
{
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      fprintf(file, "%.5f ", M[cols*i+j]);
    }
    fprintf(file, endline);
  }
}
/// \endcond

bool writeCalibrationIni(const std::string& file_name, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info)
{
  FILE* file = fopen(file_name.c_str(), "w");
  if (!file)
    return false;
  boost::shared_ptr<FILE> guard(file, fclose);

  fprintf(file, "# Camera intrinsics\n\n");
  /// @todo time?
  fprintf(file, "[image]\n\n");
  fprintf(file, "width\n%d\n\n", cam_info.width);
  fprintf(file, "height\n%d\n\n", cam_info.height);
  fprintf(file, "[%s]\n\n", camera_name.c_str());

  fprintf(file, "camera matrix\n");
  printMatrix(file, &cam_info.K[0], 3, 3);

  fprintf(file, "\ndistortion\n");
  printMatrix(file, &cam_info.D[0], 1, 5);
  
  fprintf(file, "\n\nrectification\n");
  printMatrix(file, &cam_info.R[0], 3, 3);

  fprintf(file, "\nprojection\n");
  printMatrix(file, &cam_info.P[0], 3, 4);

  return true;
}

/// \cond
// Semantic action to store a sequence of values in an array
template <typename T>
struct ArrayAssignActor
{
  ArrayAssignActor(T* start)
    : ptr_(start)
  {}

  void operator()(T val) const
  {
    *ptr_++ = val;
  }

  mutable T* ptr_;
};

// Semantic action generator
template <typename T>
ArrayAssignActor<T> array_assign_a(T* start)
{
  return ArrayAssignActor<T>(start);
}

template <typename Iterator>
bool parseCalibrationIniRange(Iterator first, Iterator last,
                              std::string& camera_name, sensor_msgs::CameraInfo& cam_info)
{
  bool have_externals = false;
  double trans[3], rot[3];

  /// @todo separate grammar out into separate function
  
  // Image section (width, height)
  BOOST_AUTO(image,
      str_p("[image]")
      >> "width"
      >> uint_p[assign_a(cam_info.width)]
      >> "height"
      >> uint_p[assign_a(cam_info.height)]
     );

  // Optional externals section
  BOOST_AUTO(externals,
      str_p("[externals]")
      >> "translation"
      >> repeat_p(3)[real_p[array_assign_a(trans)]]
      >> "rotation"
      >> repeat_p(3)[real_p[array_assign_a(rot)]]
     );

  // Parser to save name of camera section
  BOOST_AUTO(name, confix_p('[', (*anychar_p)[assign_a(camera_name)], ']'));

  // Camera section (intrinsics)
  BOOST_AUTO(camera,
      name
      >> "camera matrix"
      >> repeat_p(9)[real_p[array_assign_a(&cam_info.K[0])]]
      >> "distortion"
      >> repeat_p(5)[real_p[array_assign_a(&cam_info.D[0])]]
      >> "rectification"
      >> repeat_p(9)[real_p[array_assign_a(&cam_info.R[0])]]
      >> "projection"
      >> repeat_p(12)[real_p[array_assign_a(&cam_info.P[0])]]
     );

  // Full grammar
  BOOST_AUTO(ini_grammar,
      image
      >> !externals[assign_a(have_externals, true)]
      >>  camera);

  // Skip whitespace and line comments
  BOOST_AUTO(skip, space_p | comment_p('#'));

  parse_info<Iterator> info = parse(first, last, ini_grammar, skip);
  /// @todo Do something with externals?
  return info.hit;
}
/// \endcond

bool readCalibrationIni(const std::string& file_name, std::string& camera_name,
                        sensor_msgs::CameraInfo& cam_info)
{
  typedef file_iterator<char> Iterator;

  Iterator first(file_name);
  Iterator last = first.make_end();

  return parseCalibrationIniRange(first, last, camera_name, cam_info);
}

bool parseCalibrationIni(const std::string& buffer, std::string& camera_name,
                         sensor_msgs::CameraInfo& cam_info)
{
  return parseCalibrationIniRange(buffer.begin(), buffer.end(), camera_name, cam_info);
}

} //namespace camera_calibration_parsers
