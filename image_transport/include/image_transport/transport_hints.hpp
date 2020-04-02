/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef IMAGE_TRANSPORT__TRANSPORT_HINTS_HPP_
#define IMAGE_TRANSPORT__TRANSPORT_HINTS_HPP_

#include <memory>
#include <string>

#include <rclcpp/node.hpp>

#include "image_transport/visibility_control.hpp"

namespace image_transport
{

/**
 * \brief Stores transport settings for an image topic subscription.
 */
class TransportHints
{
public:
  /**
   * \brief Constructor.
   *
   * The default transport can be overridden by setting a certain parameter to the
   * name of the desired transport. By default this parameter is named "image_transport"
   * in the node's local namespace. For consistency across ROS applications, the
   * name of this parameter should not be changed without good reason.
   *
   * @param node Node to use when looking up the transport parameter.
   * @param default_transport Preferred transport to use
   * @param parameter_name The name of the transport parameter
   */
  IMAGE_TRANSPORT_PUBLIC
  TransportHints(
    const rclcpp::Node * node,
    const std::string & default_transport = "raw",
    const std::string & parameter_name = "image_transport")
  {
    node->get_parameter_or<std::string>(parameter_name, transport_, default_transport);
  }

  IMAGE_TRANSPORT_PUBLIC
  const std::string & getTransport() const
  {
    return transport_;
  }

private:
  std::string transport_;
};

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__TRANSPORT_HINTS_HPP_
