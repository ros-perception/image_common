// Copyright (c) 2018 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <typeinfo>

#include "image_transport/camera_common.hpp"

TEST(CameraCommon, getCameraInfoTopic_namespaced_topic) {
  const auto topic_name = "/this/is/a/topic";
  const auto info_topic = image_transport::getCameraInfoTopic(topic_name);
  EXPECT_EQ("/this/is/a/camera_info", info_topic);
}

TEST(CameraCommon, getCameraInfoTopic_topic) {
  const auto topic_name = "/topic";
  const auto info_topic = image_transport::getCameraInfoTopic(topic_name);
  EXPECT_EQ("/camera_info", info_topic);
}

// Crashes in boost implementation
TEST(CameraCommon, getCameraInfoTopic2_slash) {
  // TODO(anyone): Check if this is the correct behavior
  const auto topic_name = "/";
  const auto info_topic = image_transport::getCameraInfoTopic(topic_name);
  EXPECT_EQ("/camera_info", info_topic);
}

// Crashes in boost implementation
TEST(CameraCommon, getCameraInfoTopic2_empty) {
  // TODO(anyone): Check if this is the correct behavior
  const auto topic_name = "";
  const auto info_topic = image_transport::getCameraInfoTopic(topic_name);
  EXPECT_EQ("/camera_info", info_topic);
}

TEST(DefaultPluginsXml, raw_pub_transport_name) {
  const std::string transport = image_transport::get_transport_name_from_manifest(
    DEFAULT_PLUGINS_XML, "image_transport/raw_pub");
  EXPECT_EQ("raw", transport);
}

TEST(DefaultPluginsXml, raw_sub_transport_name) {
  const std::string transport = image_transport::get_transport_name_from_manifest(
    DEFAULT_PLUGINS_XML, "image_transport/raw_sub");
  EXPECT_EQ("raw", transport);
}

TEST(DefaultPluginsXml, raw_pub_message_type) {
  const std::string msg_type = image_transport::get_message_type_from_manifest(
    DEFAULT_PLUGINS_XML, "image_transport/raw_pub");
  EXPECT_EQ("sensor_msgs/msg/Image", msg_type);
}

TEST(DefaultPluginsXml, raw_sub_message_type) {
  const std::string msg_type = image_transport::get_message_type_from_manifest(
    DEFAULT_PLUGINS_XML, "image_transport/raw_sub");
  EXPECT_EQ("sensor_msgs/msg/Image", msg_type);
}

TEST(DefaultPluginsXml, unknown_lookup_name_returns_empty) {
  EXPECT_EQ(
    image_transport::get_transport_name_from_manifest(
      DEFAULT_PLUGINS_XML, "image_transport/does_not_exist"),
    "");
  EXPECT_EQ(
    image_transport::get_message_type_from_manifest(
      DEFAULT_PLUGINS_XML, "image_transport/does_not_exist"),
    "");
}

TEST(DefaultPluginsXml, bad_path_returns_empty) {
  EXPECT_EQ(
    image_transport::get_transport_name_from_manifest(
      "/nonexistent/path/plugins.xml", "image_transport/raw_pub"),
    "");
  EXPECT_EQ(
    image_transport::get_message_type_from_manifest(
      "/nonexistent/path/plugins.xml", "image_transport/raw_pub"),
    "");
}

TEST(CameraCommon, erase_last_copy) {
  EXPECT_EQ("image", image_transport::erase_last_copy("image_pub", "_pub"));
  EXPECT_EQ("/image_pub/image", image_transport::erase_last_copy("/image_pub/image_pub", "_pub"));
  EXPECT_EQ("/image/image", image_transport::erase_last_copy("/image_pub/image", "_pub"));
}

struct A
{
  virtual ~A() = default;
};

struct B : A {};

TEST(CameraCommon, demangle) {
  using image_transport::demangle_cpp_type_name;
  using image_transport::PluginManifestData;

  EXPECT_EQ("int", demangle_cpp_type_name(typeid(int).name()));  // NOLINT(readability/casting)
  EXPECT_EQ("image_transport::PluginManifestData",
    demangle_cpp_type_name(typeid(PluginManifestData{}).name()));
  EXPECT_EQ("A", demangle_cpp_type_name(typeid(A{}).name()));
  EXPECT_EQ("B", demangle_cpp_type_name(typeid(B{}).name()));
  A * a = new B;
  EXPECT_EQ("B", demangle_cpp_type_name(typeid(*a).name()));
  delete a;
}

// ---------------------------------------------------------------------------
// Tests that verify per-class overrides take precedence over library defaults.
// TEST_PLUGIN_MANIFEST_XML is injected as a compile definition from CMakeLists.txt.
// ---------------------------------------------------------------------------

TEST(PluginManifestXml, class_level_transport_name_overrides_library) {
  EXPECT_EQ(
    "class_transport",
    image_transport::get_transport_name_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "image_transport/class_override_pub"));
}

TEST(PluginManifestXml, class_level_message_type_overrides_library) {
  EXPECT_EQ(
    "sensor_msgs/msg/CompressedImage",
    image_transport::get_message_type_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "image_transport/class_override_pub"));
}

TEST(PluginManifestXml, library_level_transport_name_used_as_fallback) {
  EXPECT_EQ(
    "lib_transport",
    image_transport::get_transport_name_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "image_transport/lib_fallback_pub"));
}

TEST(PluginManifestXml, library_level_message_type_used_as_fallback) {
  EXPECT_EQ(
    "sensor_msgs/msg/Image",
    image_transport::get_message_type_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "image_transport/lib_fallback_pub"));
}

// ---------------------------------------------------------------------------
// Tests that verify fallback to lookup name parsing.
// TEST_PLUGIN_MANIFEST_XML is injected as a compile definition from CMakeLists.txt.
// ---------------------------------------------------------------------------

TEST(PluginManifestXml, fallback_to_lookup_name_pub) {
  EXPECT_EQ(
    "lookup",
    image_transport::get_transport_name_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "image_transport/lookup_pub"));
}

TEST(PluginManifestXml, fallback_to_lookup_name_sub) {
  EXPECT_EQ(
    "lookup",
    image_transport::get_transport_name_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "image_transport/lookup_sub"));
}
