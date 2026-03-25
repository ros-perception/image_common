// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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
//    * Neither the name of the Open Source Robotics Foundation nor the names of its
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

#include "image_transport/publisher_plugin.hpp"

#include <string>
#include <typeinfo>

#include "image_transport/camera_common.hpp"

namespace image_transport
{

// ---------------------------------------------------------------------------
// Helpers shared by both getTransportName() and getMessageType().
// ---------------------------------------------------------------------------

/// Run the manifest search if not already done and store results in the cache.
static void ensure_manifest_data(
  bool & initialized,
  std::string & transport_name_out,
  std::string & message_type_out,
  const char * mangled_this_type)
{
  if (initialized) {
    return;
  }
  initialized = true;
  const std::string demangled = demangle_cpp_type_name(mangled_this_type);
  const PluginManifestData data = get_pub_manifest_data_from_class_type(demangled);
  transport_name_out = data.transport_name;
  // Derive a default transport name from the lookup name when the manifest
  // does not declare <transport_name> (e.g. "image_transport/raw_pub" -> "raw").
  if (transport_name_out.empty() && !data.lookup_name.empty()) {
    const auto pos = data.lookup_name.rfind('/');
    const std::string short_name = (pos != std::string::npos) ?
      data.lookup_name.substr(pos + 1) :
      data.lookup_name;
    transport_name_out = erase_last_copy(short_name, "_pub");
  }
  message_type_out = data.message_type;
}

std::string PublisherPlugin::getTransportName() const
{
  ensure_manifest_data(
    manifest_data_initialized_,
    manifest_transport_name_,
    manifest_message_type_,
    typeid(*this).name());

  return manifest_transport_name_;
}

std::string PublisherPlugin::getMessageType() const
{
  ensure_manifest_data(
    manifest_data_initialized_,
    manifest_transport_name_,
    manifest_message_type_,
    typeid(*this).name());

  return manifest_message_type_;
}

}  // namespace image_transport
