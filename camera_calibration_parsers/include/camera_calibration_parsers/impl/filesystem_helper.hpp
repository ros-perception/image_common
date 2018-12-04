/*
 * Copyright (c) 2018, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/// Includes std::filesystem and aliases the namespace to `camera_calibration_parsers::impl::fs`.
/**
 * If std::filesystem is not available the necessary functions are emulated.
 */

#ifndef CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSTEM_HELPER_HPP_
#define CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSTEM_HELPER_HPP_

#if defined(_MSC_VER)
# if _MSC_VER >= 1900
#  include <experimental/filesystem>
namespace camera_calibration_parsers
{
namespace impl
{
namespace fs = std::experimental::filesystem;
}  // namespace impl
}  // namespace camera_calibration_parsers

#  define CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSYEM_HELPER__HAS_STD_FILESYSTEM
# endif

#elif defined(__has_include)
// *INDENT-OFF*
# if __has_include(<filesystem>) && __cplusplus >= 201703L  // NOLINT
// *INDENT-ON*
#  include <filesystem>

namespace camera_calibration_parsers
{
namespace impl
{
namespace fs = std::filesystem;
}  // namespace impl
}  // namespace camera_calibration_parsers

#  define CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSYEM_HELPER__HAS_STD_FILESYSTEM
// *INDENT-OFF*
# elif __has_include(<experimental/filesystem>)  // NOLINT
// *INDENT-ON*
#  include <experimental/filesystem>

namespace camera_calibration_parsers
{
namespace impl
{
namespace fs = std::experimental::filesystem;
}  // namespace impl
}  // namespace camera_calibration_parsers

#  define CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSYEM_HELPER__HAS_STD_FILESYSTEM
# endif
#endif

// The standard library does not provide it, so emulate it.
#ifndef CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSYEM_HELPER__HAS_STD_FILESYSTEM

#include <sys/stat.h>
#include <sys/types.h>

#include <string>
#include <vector>

#ifdef _WIN32
#define CLASS_LOADER_IMPL_OS_DIRSEP '\\'
#else
#define CLASS_LOADER_IMPL_OS_DIRSEP '/'
#endif

#ifdef _WIN32
  #include <io.h>
  #define access _access_s
#else
  #include <unistd.h>
#endif

#include "./split.hpp"

namespace camera_calibration_parsers
{
namespace impl
{
namespace fs
{

class path
{
public:
  static constexpr char preferred_separator = CLASS_LOADER_IMPL_OS_DIRSEP;

  path()
  : path("")
  {}

  path(const std::string & p)  // NOLINT(runtime/explicit): this is a conversion constructor
  : path_(p), path_as_vector_(split(p, std::string(1, preferred_separator)))
  {}

  std::string string() const
  {
    return path_;
  }

  bool exists() const
  {
    return access(path_.c_str(), 0) == 0;
  }

  bool empty() const
  {
    return path_ == "";
  }

  std::vector<std::string>::const_iterator cbegin() const
  {
    return path_as_vector_.cbegin();
  }

  std::vector<std::string>::const_iterator cend() const
  {
    return path_as_vector_.cend();
  }

  path parent_path() const
  {
    path parent("");
    for (auto it = this->cbegin(); it != --this->cend(); ++it) {
      parent /= *it;
    }
    return parent;
  }

  path filename() const
  {
    return path_.empty() ? path() : *--this->cend();
  }

  std::string extension() const
  {
    auto fname = filename();
    auto split_fname = split(fname.string(), std::string(1, '.'));
    return "." + *split_fname.end();
  }

  path operator/(const std::string & other)
  {
    return this->operator/(path(other));
  }

  path & operator/=(const std::string & other)
  {
    this->operator/=(path(other));
    return *this;
  }

  path operator/(const path & other)
  {
    return path(*this).operator/=(other);
  }

  path & operator/=(const path & other)
  {
    this->path_ += CLASS_LOADER_IMPL_OS_DIRSEP + other.string();
    this->path_as_vector_.insert(
      std::end(this->path_as_vector_),
      std::begin(other.path_as_vector_), std::end(other.path_as_vector_));
    return *this;
  }

private:
  std::string path_;
  std::vector<std::string> path_as_vector_;
};

inline bool create_directories(const path & p)
{
  path p_built;

  for (auto it = p.cbegin(); it != p.cend(); ++it) {
    p_built /= *it;

#ifdef _WIN32
    _mkdir(p_built.string().c_str());
#else
    mkdir(p_built.string().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
  }
  return true;
}

inline bool exists(const path & path_to_check)
{
  return path_to_check.exists();
}

#undef CLASS_LOADER_IMPL_OS_DIRSEP

}  // namespace fs
}  // namespace impl
}  // namespace camera_calibration_parsers

#endif  // CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSYEM_HELPER__HAS_STD_FILESYSTEM

#undef CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSYEM_HELPER__HAS_STD_FILESYSTEM

#endif  // CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSTEM_HELPER_HPP_
