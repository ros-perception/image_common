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

#include "image_transport/subscriber_plugin.hpp"

#include <optional>
#include <string>
#include <typeinfo>

#include "image_transport/camera_common.hpp"

namespace image_transport
{

// ---------------------------------------------------------------------------
// Helpers shared by both getTransportName() and getMessageType().
// ---------------------------------------------------------------------------

/// Populate the manifest cache on first call and return a reference to it.
static const PluginManifestData & ensure_manifest_data(
  std::optional<PluginManifestData> & cache,
  const char * mangled_this_type)
{
  if (!cache) {
    const std::string demangled = demangle_cpp_type_name(mangled_this_type);
    cache.emplace(get_sub_manifest_data_from_class_type(demangled));
  }
  return *cache;
}

std::string SubscriberPlugin::getTransportName() const
{
  return ensure_manifest_data(manifest_data_, typeid(*this).name()).transport_name;
}

std::string SubscriberPlugin::getMessageType() const
{
  return ensure_manifest_data(manifest_data_, typeid(*this).name()).message_type;
}

}  // namespace image_transport
