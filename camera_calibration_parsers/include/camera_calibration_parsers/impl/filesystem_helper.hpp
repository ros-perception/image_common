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
/// Copied from pluginlib/impl/filesystem_helper.hpp
/// Includes std::filesystem and aliases the namespace to `camera_calibration_parsers::impl::fs`.

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

# endif

#elif defined(__has_include)
# if __has_include(<filesystem>) && __cplusplus >= 201703L  // NOLINT
#  include <filesystem>

namespace camera_calibration_parsers
{
namespace impl
{
namespace fs = std::filesystem;
}  // namespace impl
}  // namespace camera_calibration_parsers

# elif __has_include(<experimental/filesystem>)  // NOLINT
#  include <experimental/filesystem>

namespace camera_calibration_parsers
{
namespace impl
{
namespace fs = std::experimental::filesystem;
}  // namespace impl
}  // namespace camera_calibration_parsers

# endif
#endif

#endif  // CAMERA_CALIBRATION_PARSERS__IMPL__FILESYSTEM_HELPER_HPP_
