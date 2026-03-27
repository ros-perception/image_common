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

#include "image_transport/camera_common.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__GNUC__) || defined(__clang__)
#include <cxxabi.h>
#endif

#include "pluginlib/class_loader.hpp"
#include "tinyxml2.h"  // NOLINT(build/include_subdir)

// Forward declarations — full definitions are not needed here, but the
// ClassLoader template requires the complete type at instantiation.
#include "image_transport/publisher_plugin.hpp"
#include "image_transport/subscriber_plugin.hpp"

namespace image_transport
{

std::vector<std::string> split(
  std::string input,
  const std::string & delim)
{
  size_t pos = 0;
  std::vector<std::string> out;

  while ((pos = input.find(delim)) != std::string::npos) {
    auto token = input.substr(0, pos);
    if (token.size() > 0) {
      out.push_back(token);
    }
    input.erase(0, pos + delim.length());
  }
  out.push_back(input);

  return out;
}

std::string getCameraInfoTopic(const std::string & base_topic)
{
  std::string info_topic;
  auto tokens = split(base_topic, "/");

  if (tokens.size() > 0) {
    for (size_t ii = 0; ii < tokens.size() - 1; ++ii) {
      info_topic.append("/");
      info_topic.append(tokens[ii]);
    }
  }
  info_topic += "/camera_info";

  return info_topic;
}

std::string erase_last_copy(const std::string & input, const std::string & search)
{
  size_t found = input.rfind(search);
  auto input_copy = input;
  if (found != std::string::npos) {
    input_copy.replace(found, search.length(), "");
  }
  return input_copy;
}

namespace
{
/// Extract the text content of a child element named @child_name.
const char * get_child_text(
  tinyxml2::XMLElement * elem,
  const char * child_name)
{
  auto * child = elem->FirstChildElement(child_name);
  return child ? child->GetText() : nullptr;
}

/// Return the first <library> element to iterate from, handling both
/// <library> and <class_libraries> as the document root.
tinyxml2::XMLElement * first_library(tinyxml2::XMLDocument & doc)
{
  auto * root = doc.RootElement();
  if (!root) {
    return nullptr;
  }
  if (std::string(root->Name()) == "class_libraries") {
    return root->FirstChildElement("library");
  }
  if (std::string(root->Name()) == "library") {
    return root;
  }
  return nullptr;
}
}  // namespace

std::string get_transport_name_from_manifest(
  const std::string & manifest_path,
  const std::string & lookup_name)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(manifest_path.c_str()) != tinyxml2::XML_SUCCESS) {
    return "";
  }
  for (auto * lib = first_library(doc); lib != nullptr; lib = lib->NextSiblingElement("library")) {
    // Library-level <transport_name> acts as the default for all classes in this library.
    const char * lib_transport = get_child_text(lib, "transport_name");
    for (auto * cls = lib->FirstChildElement("class");
      cls != nullptr;
      cls = cls->NextSiblingElement("class"))
    {
      const char * name = cls->Attribute("name");
      if (!name || lookup_name != name) {
        continue;
      }
      // Prefer a <transport_name> on the <class> itself; fall back to the library-level one.
      const char * cls_transport = get_child_text(cls, "transport_name");
      if (cls_transport) {
        return cls_transport;
      }
      if (lib_transport) {
        return lib_transport;
      }
      // Derive a default transport name from the lookup name when the manifest
      // does not declare <transport_name> (e.g. "image_transport/raw_sub" -> "raw").
      if (!lookup_name.empty()) {
        const auto pos = lookup_name.rfind('/');
        const std::string short_name = (pos != std::string::npos) ?
          lookup_name.substr(pos + 1) :
          lookup_name;
        auto lookup_name_transport = erase_last_copy(short_name, "_sub");
        lookup_name_transport = erase_last_copy(lookup_name_transport, "_pub");
        return lookup_name_transport;
      }
      return "";
    }
  }
  return "";
}

std::string get_message_type_from_manifest(
  const std::string & manifest_path,
  const std::string & lookup_name)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(manifest_path.c_str()) != tinyxml2::XML_SUCCESS) {
    return "";
  }
  for (auto * lib = first_library(doc); lib != nullptr; lib = lib->NextSiblingElement("library")) {
    // Library-level <message_type> acts as the default for all classes in this library.
    const char * lib_type = get_child_text(lib, "message_type");
    for (auto * cls = lib->FirstChildElement("class");
      cls != nullptr;
      cls = cls->NextSiblingElement("class"))
    {
      const char * name = cls->Attribute("name");
      if (!name || lookup_name != name) {
        continue;
      }
      // Prefer a <message_type> on the <class> itself; fall back to the library-level one.
      const char * cls_type = get_child_text(cls, "message_type");
      if (cls_type) {
        return cls_type;
      }
      return lib_type ? lib_type : "";
    }
  }
  return "";
}

}  // namespace image_transport

// ---------------------------------------------------------------------------
// Manifest auto-discovery helpers
// ---------------------------------------------------------------------------

namespace
{

template<class BaseT>
image_transport::PluginManifestData search_manifest_for_type(
  const std::string & package,
  const std::string & base_class_type,
  const std::string & cpp_type_name)
{
  try {
    pluginlib::ClassLoader<BaseT> loader(package, base_class_type);
    for (const auto & lookup_name : loader.getDeclaredClasses()) {
      if (loader.getClassType(lookup_name) == cpp_type_name) {
        const std::string manifest_path = loader.getPluginManifestPath(lookup_name);
        return {
          image_transport::get_transport_name_from_manifest(manifest_path, lookup_name),
          image_transport::get_message_type_from_manifest(manifest_path, lookup_name),
          lookup_name
        };
      }
    }
  } catch (const std::exception &) {
    // Silently ignore: ament index unavailable, no plugins registered, etc.
  }
  return {};
}

}  // anonymous namespace

namespace image_transport
{

std::string demangle_cpp_type_name(const char * mangled_name)
{
#if defined(__GNUC__) || defined(__clang__)
  int status = 0;
  char * d = abi::__cxa_demangle(mangled_name, nullptr, nullptr, &status);
  std::string result = (status == 0 && d) ? d : mangled_name;
  std::free(d);
  return result;
#elif defined(_MSC_VER)
  // MSVC's typeid().name() is already human-readable, but prepends 'class ' or 'struct '
  std::string result = mangled_name;
  if (result.size() > 6 && result.substr(0, 6) == "class ") {
    result = result.substr(6);
  }
  if (result.size() > 7 && result.substr(0, 7) == "struct ") {
    result = result.substr(7);
  }
  return result;
#else
  return mangled_name;
#endif
}

PluginManifestData get_pub_manifest_data_from_class_type(const std::string & cpp_type_name)
{
  return search_manifest_for_type<PublisherPlugin>(
    "image_transport", "image_transport::PublisherPlugin", cpp_type_name);
}

PluginManifestData get_sub_manifest_data_from_class_type(const std::string & cpp_type_name)
{
  return search_manifest_for_type<SubscriberPlugin>(
    "image_transport", "image_transport::SubscriberPlugin", cpp_type_name);
}

}  // namespace image_transport
