// Copyright (c) 2023 Open Source Robotics Foundation, Inc.
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

#include "image_transport/republish.hpp"

#include <rclcpp/rclcpp.hpp>

#include "utilities/utilities.hpp"

int main(int argc, char ** argv)
{
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  // remove program name
  args.erase(args.begin());

  std::string in_transport{"raw"};
  std::string out_transport{""};

  if (image_transport::has_option(args, "--in_transport")) {
    in_transport = image_transport::get_option(args, "--in_transport");
  }
  if (image_transport::has_option(args, "--out_transport")) {
    out_transport = image_transport::get_option(args, "--out_transport");
  }

  rclcpp::NodeOptions options;
  // override default parameters with the desired transform
  options.parameter_overrides(
  {
    {"in_transport", in_transport},
    {"out_transport", out_transport},
  });

  std::shared_ptr<image_transport::Republisher> node;

  node = std::make_shared<image_transport::Republisher>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();
}
