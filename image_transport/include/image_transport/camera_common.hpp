// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef IMAGE_TRANSPORT__CAMERA_COMMON_HPP_
#define IMAGE_TRANSPORT__CAMERA_COMMON_HPP_

#include <string>

#include "image_transport/visibility_control.hpp"

namespace image_transport
{

/// Data read from a plugin manifest XML for one class.
struct PluginManifestData
{
  std::string transport_name;  ///< e.g. "raw"
  std::string message_type;    ///< e.g. "sensor_msgs/msg/Image"
  std::string lookup_name;     ///< e.g. "image_transport/raw_pub"
};

/**
 * \brief Demangle a C++ ABI type name to its human-readable form.
 *
 * On GCC/Clang this calls __cxa_demangle; on other platforms the
 * mangled name is returned unchanged.
 */
IMAGE_TRANSPORT_PUBLIC
std::string demangle_cpp_type_name(const char * mangled_name);

/**
 * \brief Search the pluginlib publisher-plugin registry for the class whose
 * concrete (derived) C++ type equals \p cpp_type_name and return its manifest
 * data (transport name, message type, lookup name).
 *
 * The search scans all plugin XML files registered under the "image_transport"
 * package for classes derived from image_transport::PublisherPlugin.  No plugin
 * is instantiated; the function only parses XML.  Returns an empty
 * PluginManifestData if no match is found.
 */
IMAGE_TRANSPORT_PUBLIC
PluginManifestData get_pub_manifest_data_from_class_type(const std::string & cpp_type_name);

/**
 * \brief Search the pluginlib subscriber-plugin registry for the class whose
 * concrete (derived) C++ type equals \p cpp_type_name and return its manifest
 * data (transport name, message type, lookup name).
 *
 * Analogous to get_pub_manifest_data_from_class_type but searches
 * image_transport::SubscriberPlugin classes.
 */
IMAGE_TRANSPORT_PUBLIC
PluginManifestData get_sub_manifest_data_from_class_type(const std::string & cpp_type_name);

/**
 * \brief Form the camera info topic name, sibling to the base topic.
 *
 * \note This function assumes that the name is completely resolved. If the \c
 * base_topic is remapped the resulting camera info topic will be incorrect.
 */
IMAGE_TRANSPORT_PUBLIC
std::string getCameraInfoTopic(const std::string & base_topic);

/**
 * \brief Replacement for uses of boost::erase_last_copy
 */
IMAGE_TRANSPORT_PUBLIC
std::string erase_last_copy(const std::string & input, const std::string & search);

/**
 * \brief Read the transport name declared in a plugin manifest XML for a given
 * class lookup name.
 *
 * Parses the \c <transport_name name="..."/> child element of the matching
 * \c <class> entry without instantiating the plugin.
 *
 * \param manifest_path Absolute path to the plugin XML manifest file.
 * \param lookup_name  The \c name attribute of the target \c <class> element.
 * \return The transport name string (e.g. "raw"), or an empty string
 *         if the element is absent or the file cannot be parsed.
 */
IMAGE_TRANSPORT_PUBLIC
std::string get_transport_name_from_manifest(
  const std::string & manifest_path,
  const std::string & lookup_name);

/**
 * \brief Read the message type declared in a plugin manifest XML for a given
 * class lookup name.
 *
 * This should be a "representative" message type for this plugin. If it
 * communicates over a single topic (the most common case), use
 * the type of this topic.
 *
 * Parses the \c <message_type> child element of the enclosing \c <library>
 * element without instantiating the plugin.  The element is shared by all
 * classes in the same library, mirroring how \c <transport_name> works.
 *
 * \param manifest_path Absolute path to the plugin XML manifest file.
 * \param lookup_name  The \c name attribute of the target \c <class> element.
 * \return The type string (e.g. "sensor_msgs/msg/Image"), or an empty string
 *         if the element is absent or the file cannot be parsed.
 */
IMAGE_TRANSPORT_PUBLIC
std::string get_message_type_from_manifest(
  const std::string & manifest_path,
  const std::string & lookup_name);

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__CAMERA_COMMON_HPP_
