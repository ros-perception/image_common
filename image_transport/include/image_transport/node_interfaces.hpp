// Copyright (c) 2025, Open Source Robotics Foundation, Inc.
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

#ifndef IMAGE_TRANSPORT__NODE_INTERFACES_HPP_
#define IMAGE_TRANSPORT__NODE_INTERFACES_HPP_

#include <rclcpp/node_interfaces/node_interfaces.hpp>

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

namespace image_transport
{
using NodeBaseInterface = rclcpp::node_interfaces::NodeBaseInterface;
using NodeParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;
using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;
using NodeTimersInterface = rclcpp::node_interfaces::NodeTimersInterface;
using NodeTopicsInterface = rclcpp::node_interfaces::NodeTopicsInterface;

using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
  NodeBaseInterface,
  NodeParametersInterface,
  NodeLoggingInterface,
  NodeTimersInterface,
  NodeTopicsInterface>;
}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__NODE_INTERFACES_HPP_
