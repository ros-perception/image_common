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
 * Author: Ryohei Ueda <garaemon@gmail.com>
 *         Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#ifndef IMAGE_TRANSPORT_NODELET_LAZY_H__
#define IMAGE_TRANSPORT_NODELET_LAZY_H__


#include <image_transport/image_transport.h>
#include <image_transport/publisher_plugin.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
#include <pluginlib/class_loader.h>

/** @brief
 * Nodelet to automatically subscribe/unsubscribe
 * image/camera topics according to subscription of advertised topics.
 *
 * It's important not to subscribe topic if no output is required.
 *
 * In order to watch advertised topics, need to use advertise template method.
 * And create subscribers in subscribe() and shutdown them in unsubscribed().
 *
 */
namespace image_transport
{
class NodeletLazy : public nodelet_topic_tools::NodeletLazy
{
protected:

  /** @brief
    * callback function which is called when a new subscriber comes
    */
  virtual void imageConnectionCallback(
      const SingleSubscriberPublisher&)
  {
    if (verbose_connection_)
    {
      NODELET_INFO("New image connection or disconnection is detected");
    }
    if (lazy_)
    {
      boost::mutex::scoped_lock lock(connection_mutex_);
      for (size_t i = 0; i < image_publishers_.size(); i++)
      {
        Publisher pub = image_publishers_[i];
        if (pub.getNumSubscribers() > 0)
        {
          if (connection_status_ != nodelet_topic_tools::SUBSCRIBED)
          {
            if (verbose_connection_)
            {
              NODELET_INFO("Subscribe input topics");
            }
            subscribe();
            connection_status_ = nodelet_topic_tools::SUBSCRIBED;
          }
          if (!ever_subscribed_)
          {
            ever_subscribed_ = true;
          }
          return;
        }
      }
      if (connection_status_ == nodelet_topic_tools::SUBSCRIBED)
      {
        if (verbose_connection_)
        {
          NODELET_INFO("Unsubscribe input topics");
        }
        unsubscribe();
        connection_status_ = nodelet_topic_tools::NOT_SUBSCRIBED;
      }
    }
  }

  /** @brief
    * callback function which is called when a new subscriber comes
    */
  virtual void imagePluginConnectionCallback(
      const SingleSubscriberPublisher&)
  {
    if (verbose_connection_)
    {
      NODELET_INFO("New image connection or disconnection is detected");
    }
    if (lazy_)
    {
      boost::mutex::scoped_lock lock(connection_mutex_);
      for (size_t i = 0; i < image_publisher_plugins_.size(); i++)
      {
        boost::weak_ptr<PublisherPlugin>
            weak_pub = image_publisher_plugins_[i];
        if (boost::shared_ptr<PublisherPlugin> pub = weak_pub.lock())
        {
          if (pub->getNumSubscribers() > 0)
          {
            if (connection_status_ != nodelet_topic_tools::SUBSCRIBED)
            {
              if (verbose_connection_)
              {
                NODELET_INFO("Subscribe input topics");
              }
              subscribe();
              connection_status_ = nodelet_topic_tools::SUBSCRIBED;
            }
            if (!ever_subscribed_)
            {
              ever_subscribed_ = true;
            }
            return;
          }
        }
        else
        {
          NODELET_ERROR("Image Plugin is already deallocated");
        }
      }
      if (connection_status_ == nodelet_topic_tools::SUBSCRIBED)
      {
        if (verbose_connection_)
        {
          NODELET_INFO("Unsubscribe input topics");
        }
        unsubscribe();
        connection_status_ = nodelet_topic_tools::NOT_SUBSCRIBED;
      }
    }
  }

  void cameraConnectionCallbackImpl()
  {
    if (verbose_connection_)
    {
      NODELET_INFO("New camera connection or disconnection is detected");
    }
    if (lazy_)
    {
      boost::mutex::scoped_lock lock(connection_mutex_);
      for (size_t i = 0; i < camera_publishers_.size(); i++)
      {
        CameraPublisher pub = camera_publishers_[i];
        if (pub.getNumSubscribers() > 0)
        {
          if (connection_status_ != nodelet_topic_tools::SUBSCRIBED)
          {
            if (verbose_connection_)
            {
              NODELET_INFO("Subscribe input topics");
            }
            subscribe();
            connection_status_ = nodelet_topic_tools::SUBSCRIBED;
          }
          if (!ever_subscribed_)
          {
            ever_subscribed_ = true;
          }
          return;
        }
      }
      if (connection_status_ == nodelet_topic_tools::SUBSCRIBED)
      {
        if (verbose_connection_)
        {
          NODELET_INFO("Unsubscribe input topics");
        }
        unsubscribe();
        connection_status_ = nodelet_topic_tools::NOT_SUBSCRIBED;
      }
    }
  }

  /** @brief
    * callback function which is called when a new subscriber comes
    */
  virtual void cameraConnectionCallback(
      const SingleSubscriberPublisher&)
  {
    cameraConnectionCallbackImpl();
  }

  /** @brief
    * callback function which is called when a new subscriber comes
    */
  virtual void cameraInfoConnectionCallback(
      const ros::SingleSubscriberPublisher&)
  {
    cameraConnectionCallbackImpl();
  }

  /** @brief
   * Update the list of Publishers created by this method.
   * It automatically reads latch boolean parameter from nh and
   * publish topic with appropriate latch parameter.
   *
   * @param nh NodeHandle.
   * @param topic topic name to advertise.
   * @param queue_size queue size for publisher.
   * @param latch set true if latch topic publication.
   * @return Publisher for the advertised topic.
   */
  Publisher
  advertiseImage(ros::NodeHandle& nh,
                 const std::string& topic,
                 int queue_size,
                 bool latch=false)
  {
    boost::mutex::scoped_lock lock(connection_mutex_);
    SubscriberStatusCallback connect_cb
        = boost::bind(&NodeletLazy::imageConnectionCallback, this, _1);
    SubscriberStatusCallback disconnect_cb
        = boost::bind(&NodeletLazy::imageConnectionCallback, this, _1);

    Publisher pub = ImageTransport(nh).advertise(
        topic, queue_size,
        connect_cb, disconnect_cb,
        ros::VoidPtr(),
        latch);
    image_publishers_.push_back(pub);
    return pub;
  }

  /** @brief
   * Update the list of Publishers created by this method.
   * It automatically reads latch boolean parameter from nh and
   * publish topic with appropriate latch parameter.
   *
   * @param nh NodeHandle.
   * @param topic topic name to advertise.
   * @param queue_size queue size for publisher.
   * @param latch set true if latch topic publication.
   * @param transport transport name for advertised topic
   */
  boost::shared_ptr<PublisherPlugin>
  advertiseImage(ros::NodeHandle& nh,
                 const std::string& topic,
                 int queue_size,
                 boost::weak_ptr<PublisherPlugin> plugin,
                 bool latch=false)
  {
    boost::mutex::scoped_lock lock(connection_mutex_);
    SubscriberStatusCallback connect_cb
        = boost::bind(&NodeletLazy::imagePluginConnectionCallback, this, _1);
    SubscriberStatusCallback disconnect_cb
        = boost::bind(&NodeletLazy::imagePluginConnectionCallback, this, _1);
    std::string resolved_topic = nh.resolveName(topic);
    if (boost::shared_ptr<PublisherPlugin> shared_plugin = plugin.lock())
    {
      shared_plugin->advertise(
          nh, resolved_topic, queue_size,
          connect_cb, disconnect_cb,
          ros::VoidPtr(),
          latch);
      image_publisher_plugins_.push_back(plugin);
    }
  }

  /** @brief
   * Update the list of Publishers created by this method.
   * It automatically reads latch boolean parameter from nh and
   * publish topic with appropriate latch parameter.
   *
   * @param nh NodeHandle.
   * @param topic topic name to advertise.
   * @param queue_size queue size for publisher.
   * @param latch set true if latch topic publication.
   * @return CameraPublisher for the advertised topic.
   */
  CameraPublisher
  advertiseCamera(ros::NodeHandle& nh,
                  const std::string& topic,
                  int queue_size,
                  bool latch=false)
  {
    boost::mutex::scoped_lock lock(connection_mutex_);
    SubscriberStatusCallback connect_cb
        = boost::bind(&NodeletLazy::cameraConnectionCallback, this, _1);
    SubscriberStatusCallback disconnect_cb
        = boost::bind(&NodeletLazy::cameraConnectionCallback, this, _1);
    ros::SubscriberStatusCallback info_connect_cb
        = boost::bind(&NodeletLazy::cameraInfoConnectionCallback, this, _1);
    ros::SubscriberStatusCallback info_disconnect_cb
        = boost::bind(&NodeletLazy::cameraInfoConnectionCallback, this, _1);

    CameraPublisher pub = ImageTransport(nh).advertiseCamera(
        topic, queue_size,
        connect_cb, disconnect_cb,
        info_connect_cb, info_disconnect_cb,
        ros::VoidPtr(),
        latch);
    camera_publishers_.push_back(pub);
    return pub;
  }

  /** @brief
   * List of monitoring image publishers
   */
  std::vector<Publisher> image_publishers_;

  /** @brief
   * List of monitoring image publishers
   */
  std::vector<boost::weak_ptr<PublisherPlugin> > image_publisher_plugins_;

  /** @brief
   * List of monitoring camera publishers
   */
  std::vector<CameraPublisher> camera_publishers_;
};

} // namespace image_transport

#endif // IMAGE_TRANSPORT_NODELET_LAZY_H__
