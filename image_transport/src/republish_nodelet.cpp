// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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


#include <image_transport/image_transport.h>
#include <image_transport/nodelet_lazy.h>
#include <image_transport/publisher_plugin.h>
#include <pluginlib/class_loader.h>


namespace image_transport
{
class RepublishNodelet : public NodeletLazy
{
protected:
  boost::shared_ptr<ImageTransport> it_;
  Publisher pub_;
  boost::shared_ptr<pluginlib::ClassLoader<PublisherPlugin> > loader_;
  boost::shared_ptr<PublisherPlugin> pub_plugin_;
  Subscriber sub_;
  std::string in_transport_, out_transport_;

  virtual void onInit()
  {
    NodeletLazy::onInit();

    pnh_->getParam("in_transport", in_transport_);
    pnh_->param<std::string>("out_transport", out_transport_, std::string());

    it_.reset(new ImageTransport(*nh_));

    if (out_transport_.empty())
    {
      pub_ = advertiseImage(*nh_, "out", 1);
    }
    else
    {
      loader_.reset(new pluginlib::ClassLoader<PublisherPlugin>(
          "image_transport", "image_transport::PublisherPlugin"));
      std::string lookup_name = PublisherPlugin::getLookupName(out_transport_);
      pub_plugin_ = loader_->createInstance(lookup_name);

      advertiseImage(*nh_, "out", 1, boost::weak_ptr<PublisherPlugin>(pub_plugin_));
    }

    onInitPostProcess();
  }

  virtual void subscribe()
  {
    std::string in_topic = nh_->resolveName("in");
    if (out_transport_.empty())
    {
      // Use Publisher::publish as the subscriber callback
      typedef void (Publisher::*PublishMemFn)(const sensor_msgs::ImageConstPtr&) const;
      PublishMemFn pub_mem_fn = &Publisher::publish;
      sub_ = it_->subscribe(
          in_topic, 1,
          boost::bind(pub_mem_fn, &pub_, _1),
          ros::VoidPtr(), in_transport_);
    }
    else
    {
      typedef void (PublisherPlugin::*PublishMemFn)(const sensor_msgs::ImageConstPtr&) const;
      PublishMemFn pub_mem_fn = &PublisherPlugin::publish;
      sub_ = it_->subscribe(
          in_topic, 1,
          boost::bind(pub_mem_fn, pub_plugin_.get(), _1),
          pub_plugin_, in_transport_);
    }
  }

  virtual void unsubscribe()
  {
    sub_.shutdown();
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_transport::RepublishNodelet, nodelet::Nodelet)
