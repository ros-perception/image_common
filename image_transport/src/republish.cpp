// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
/*
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <nodelet/loader.h>
#include <ros/ros.h>


namespace ns {
  std::string validate(const std::string& name)
  {
    std::vector<std::string> list_all, list_nonempty;
    std::string delim("/");
    boost::split(list_all, name, boost::is_any_of(delim));
    for (std::vector<std::string>::iterator it = list_all.begin(); it != list_all.end(); ++it)
    {
      if (!it->empty()) list_nonempty.push_back(*it);
    }
    return delim + boost::algorithm::join(list_nonempty, delim);
  }

  std::string join(const std::string& parent, const std::string& child)
  {
    return validate(parent + "/" + child);
  }

  std::string basename(const std::string& name)
  {
    std::vector<std::string> list;
    boost::split(list, name,boost::is_any_of("/"));
    if (list.size() > 0) return list[list.size() - 1];
    else return name;
  }
}


void remapToPrivate(const nodelet::M_string& src, nodelet::M_string& dst)
{
  for (nodelet::M_string::const_iterator it = src.begin(); it != src.end(); ++it)
  {
    std::string key = ns::join(ros::this_node::getName(), ns::basename(it->first));
    dst[key] = it->second;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_republisher", ros::init_options::AnonymousName);

  if (argc < 2)
  {
    printf("Usage: %s in_transport in:=<in_base_topic> [out_transport] out:=<out_base_topic>\n", argv[0]);
    return 1;
  }

  // set transport
  ros::param::set("~in_transport", argv[1]);
  if (argc >= 3)
  {
    ros::param::set("~out_transport", argv[2]);
  }

  nodelet::Loader loader;
  nodelet::V_string nargv;

  // remap 'in', 'out' topic into ~in, ~out for backward compatibility
  nodelet::M_string remappings;
  remapToPrivate(ros::names::getRemappings(), remappings);

  loader.load(ros::this_node::getName(),
              "image_transport/Republish",
              remappings, nargv);

  ros::spin();

  return 0;
}
