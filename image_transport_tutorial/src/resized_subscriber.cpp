// Copyright 2021, Open Source Robotics Foundation, Inc.
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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>

#include "image_transport_tutorial/resized_subscriber.hpp"

void ResizedSubscriber::internalCallback(
  const image_transport_tutorial::msg::ResizedImage::ConstSharedPtr & msg,
  const Callback & user_cb)
{
  // This is only for optimization, not to copy the image
  std::shared_ptr<void const> tracked_object_tmp;
  cv::Mat img_rsz = cv_bridge::toCvShare(msg->image, tracked_object_tmp)->image;
  // Resize the image to its original size
  cv::Mat img_restored;
  cv::resize(img_rsz, img_restored, cv::Size(msg->original_width, msg->original_height));

  // Call the user callback with the restored image
  cv_bridge::CvImage cv_img(msg->image.header, msg->image.encoding, img_restored);
  user_cb(cv_img.toImageMsg());
}
