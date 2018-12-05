// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMAGE_TRANSPORT__VISIBILITY_CONTROL_HPP_
#define IMAGE_TRANSPORT__VISIBILITY_CONTROL_HPP_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_TRANSPORT_EXPORT __attribute__ ((dllexport))
    #define IMAGE_TRANSPORT_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_TRANSPORT_EXPORT __declspec(dllexport)
    #define IMAGE_TRANSPORT_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_TRANSPORT_BUILDING_DLL
    #define IMAGE_TRANSPORT_PUBLIC IMAGE_TRANSPORT_EXPORT
  #else
    #define IMAGE_TRANSPORT_PUBLIC IMAGE_TRANSPORT_IMPORT
  #endif
  #define IMAGE_TRANSPORT_PUBLIC_TYPE IMAGE_TRANSPORT_PUBLIC
  #define IMAGE_TRANSPORT_LOCAL
#else
  #define IMAGE_TRANSPORT_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_TRANSPORT_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_TRANSPORT_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_TRANSPORT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_TRANSPORT_PUBLIC
    #define IMAGE_TRANSPORT_LOCAL
  #endif
  #define IMAGE_TRANSPORT_PUBLIC_TYPE
#endif

#if __cplusplus
}
#endif

#endif  // IMAGE_TRANSPORT__VISIBILITY_CONTROL_HPP_
