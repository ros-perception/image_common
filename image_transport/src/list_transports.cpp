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

#include <iostream>
#include <map>
#include <string>

#include "pluginlib/class_loader.hpp"

#include "image_transport/camera_common.hpp"
#include "image_transport/publisher_plugin.hpp"
#include "image_transport/subscriber_plugin.hpp"

enum PluginStatus {SUCCESS, CREATE_FAILURE, LIB_LOAD_FAILURE, DOES_NOT_EXIST};

/// \cond
struct TransportDesc
{
  TransportDesc()
  : pub_status(DOES_NOT_EXIST), sub_status(DOES_NOT_EXIST)
  {}

  std::string package_name;
  std::string pub_name;
  PluginStatus pub_status;
  std::string pub_message_type;
  std::string sub_name;
  PluginStatus sub_status;
  std::string sub_message_type;
};
/// \endcond

int main(int /*argc*/, char ** /*argv*/)
{
  pluginlib::ClassLoader<image_transport::PublisherPlugin> pub_loader(
    "image_transport", "image_transport::PublisherPlugin");
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
    "image_transport", "image_transport::SubscriberPlugin");

  std::map<std::string, TransportDesc> transports;

  for (const std::string & lookup_name : pub_loader.getDeclaredClasses()) {
    std::string transport_name = image_transport::erase_last_copy(lookup_name, "_pub");
    transports[transport_name].pub_name = lookup_name;
    transports[transport_name].package_name = pub_loader.getClassPackage(lookup_name);
    try {
      auto pub = pub_loader.createUniqueInstance(lookup_name);
      transports[transport_name].pub_status = SUCCESS;
      transports[transport_name].pub_message_type = pub->getMessageType();
    } catch (const pluginlib::LibraryLoadException &) {
      transports[transport_name].pub_status = LIB_LOAD_FAILURE;
    } catch (const pluginlib::CreateClassException &) {
      transports[transport_name].pub_status = CREATE_FAILURE;
    }
  }

  for (const std::string & lookup_name : sub_loader.getDeclaredClasses()) {
    std::string transport_name = image_transport::erase_last_copy(lookup_name, "_sub");
    transports[transport_name].sub_name = lookup_name;
    transports[transport_name].package_name = sub_loader.getClassPackage(lookup_name);
    try {
      auto sub = sub_loader.createUniqueInstance(lookup_name);
      transports[transport_name].sub_status = SUCCESS;
      transports[transport_name].sub_message_type = sub->getMessageType();
    } catch (const pluginlib::LibraryLoadException &) {
      transports[transport_name].sub_status = LIB_LOAD_FAILURE;
    } catch (const pluginlib::CreateClassException &) {
      transports[transport_name].sub_status = CREATE_FAILURE;
    }
  }

  std::cout << "Declared transports:\n";
  for (const auto & [name, td] : transports) {
    std::cout << name;
    if ((td.pub_status == CREATE_FAILURE || td.pub_status == LIB_LOAD_FAILURE) ||
      (td.sub_status == CREATE_FAILURE || td.sub_status == LIB_LOAD_FAILURE))
    {
      std::cout << " (*): Not available. Try 'catkin_make --pkg " << td.package_name << "'.";
    }
    std::cout << "\n";
  }

  std::cout << "\nDetails:\n";
  for (const auto & [name, td] : transports) {
    std::cout << "----------\n";
    std::cout << "\"" << name << "\"\n";
    if (td.pub_status == CREATE_FAILURE || td.sub_status == CREATE_FAILURE) {
      std::cout <<
        "*** Plugins are built, but could not be loaded. The package may need to be rebuilt or may "
        "not be compatible with this release of image_common. ***\n";
    } else if (td.pub_status == LIB_LOAD_FAILURE || td.sub_status == LIB_LOAD_FAILURE) {
      std::cout << "*** Plugins are not built. ***\n";
    }
    std::cout << " - Provided by package: " << td.package_name << "\n";
    if (td.pub_status == DOES_NOT_EXIST) {
      std::cout << " - No publisher provided\n";
    } else {
      std::cout << " - Publisher: " << pub_loader.getClassDescription(td.pub_name) << "\n";
      if (!td.pub_message_type.empty()) {
        std::cout << " - Publisher message type: " << td.pub_message_type << "\n";
      }
    }
    if (td.sub_status == DOES_NOT_EXIST) {
      std::cout << " - No subscriber provided\n";
    } else {
      std::cout << " - Subscriber: " << sub_loader.getClassDescription(td.sub_name) << "\n";
      if (!td.sub_message_type.empty()) {
        std::cout << " - Subscriber message type: " << td.sub_message_type << "\n";
      }
    }
  }

  return 0;
}
