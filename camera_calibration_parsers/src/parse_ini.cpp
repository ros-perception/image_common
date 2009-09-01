#include "camera_calibration_parsers/parse_ini.h"

#include <boost/shared_ptr.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_file_iterator.hpp>
#include <boost/spirit/include/classic_confix.hpp>
#include <boost/spirit/include/classic_loops.hpp>
#include <boost/typeof/typeof.hpp>
#include <cstdio>

namespace camera_calibration_parsers {

// @todo: move to new spirit
using namespace BOOST_SPIRIT_CLASSIC_NS;

static void printMatrix(FILE* file, const double* M, int rows, int cols, const char* endline = "\n")
{
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      fprintf(file, "%.5f ", M[cols*i+j]);
    }
    fprintf(file, endline);
  }
}

bool writeCalibrationIni(const std::string& file_name, const std::string& camera_name,
                         int width, int height,
                         const double* K, const double* D,
                         const double* R, const double* P)
{
  FILE* file = fopen(file_name.c_str(), "w");
  if (!file)
    return false;
  boost::shared_ptr<FILE> guard(file, fclose);

  fprintf(file, "# Camera intrinsics\n\n");
  /** @todo: time? */
  fprintf(file, "[image]\n\n");
  fprintf(file, "width\n%d\n\n", width);
  fprintf(file, "height\n%d\n\n", height);

  fprintf(file, "[%s]\n\n", camera_name.c_str());
  fprintf(file, "camera matrix\n");
  printMatrix(file, K, 3, 3);
  
  fprintf(file, "\ndistortion\n");
  if (D) {
    printMatrix(file, D, 1, 5);
  } else {
    fprintf(file, "0.00000 0.00000 0.00000 0.00000 0.00000\n");
  }
  
  fprintf(file, "\n\nrectification\n");
  if (R) {
    printMatrix(file, R, 3, 3);
  } else {
    fprintf(file, "1.00000 0.00000 0.00000\n");
    fprintf(file, "0.00000 1.00000 0.00000\n");
    fprintf(file, "0.00000 0.00000 1.00000\n");
  }

  fprintf(file, "\nprojection\n");
  if (P) {
    printMatrix(file, P, 3, 4);
  } else {
    printMatrix(file, K, 3, 3, "0.00000\n");
  }

  return true;
}

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
                              std::string& camera_name, int& width, int& height,
                              double* K, double* D, double* R, double* P)
{
  /** @todo: actually parse only what's requested? */
  double ignore[12];
  if (!D) D = ignore;
  if (!R) R = ignore;
  if (!P) P = ignore;
  
  bool have_externals = false;
  double trans[3], rot[3];

  /** @todo: separate grammar out into separate function */
  
  // Image section (width, height)
  BOOST_AUTO(image,
      str_p("[image]")
      >> "width"
      >> uint_p[assign_a(width)]
      >> "height"
      >> uint_p[assign_a(height)]
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
      >> repeat_p(9)[real_p[array_assign_a(K)]]
      >> "distortion"
      >> repeat_p(5)[real_p[array_assign_a(D)]]
      >> "rectification"
      >> repeat_p(9)[real_p[array_assign_a(R)]]
      >> "projection"
      >> repeat_p(12)[real_p[array_assign_a(P)]]
     );

  // Full grammar
  BOOST_AUTO(ini_grammar,
      image
      >> !externals[assign_a(have_externals, true)]
      >>  camera);

  // Skip whitespace and line comments
  BOOST_AUTO(skip, space_p | comment_p('#'));

  parse_info<Iterator> info = parse(first, last, ini_grammar, skip);
  /** @todo: do anything with externals? */
  return info.hit;
}

bool readCalibrationIni(const std::string& file_name, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D, double* R, double* P)
{
  typedef file_iterator<char> Iterator;

  Iterator first(file_name);
  Iterator last = first.make_end();

  return parseCalibrationIniRange(first, last, camera_name, width, height, K, D, R, P);
}

bool parseCalibrationIni(const std::string& buffer, std::string& camera_name,
                        int &width, int &height,
                        double* K, double* D, double* R, double* P)
{
  return parseCalibrationIniRange(buffer.begin(), buffer.end(), camera_name,
                                  width, height, K, D, R, P);
}

} //namespace camera_calibration_parsers
