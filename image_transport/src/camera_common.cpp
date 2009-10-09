#include "image_transport/camera_common.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/join.hpp>
#include <vector>

namespace image_transport {

std::string getCameraInfoTopic(const std::string& base_topic)
{
  // Split into separate names
  std::vector<std::string> names;
  boost::algorithm::split(names, base_topic, boost::algorithm::is_any_of("/"),
                          boost::algorithm::token_compress_on);
  // Get rid of empty tokens from trailing slashes
  while (names.back().empty())
    names.pop_back();
  // Replace image name with "camera_info"
  names.back() = "camera_info";
  // Join back together into topic name
  return boost::algorithm::join(names, "/");
}

} //namespace image_transport
