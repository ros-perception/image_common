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

#include "image_transport/camera_common.hpp"

TEST(CameraCommon, getCameraInfoTopic_namespaced_topic) {
  const auto topic_name = "/this/is/a/topic";
  const auto info_topic = image_transport::getCameraInfoTopic(topic_name);
  EXPECT_EQ(info_topic, "/this/is/a/camera_info");
}

TEST(CameraCommon, getCameraInfoTopic_topic) {
  const auto topic_name = "/topic";
  const auto info_topic = image_transport::getCameraInfoTopic(topic_name);
  EXPECT_EQ(info_topic, "/camera_info");
}

// Crashes in boost implementation
TEST(CameraCommon, getCameraInfoTopic2_slash) {
  // TODO(anyone): Check if this is the correct behavior
  const auto topic_name = "/";
  const auto info_topic = image_transport::getCameraInfoTopic(topic_name);
  EXPECT_EQ(info_topic, "/camera_info");
}

// Crashes in boost implementation
TEST(CameraCommon, getCameraInfoTopic2_empty) {
  // TODO(anyone): Check if this is the correct behavior
  const auto topic_name = "";
  const auto info_topic = image_transport::getCameraInfoTopic(topic_name);
  EXPECT_EQ(info_topic, "/camera_info");
}

TEST(CameraCommon, erase_last_copy) {
  EXPECT_EQ("image", image_transport::erase_last_copy("image_pub", "_pub"));
  EXPECT_EQ("/image_pub/image", image_transport::erase_last_copy("/image_pub/image_pub", "_pub"));
  EXPECT_EQ("/image/image", image_transport::erase_last_copy("/image_pub/image", "_pub"));
}
