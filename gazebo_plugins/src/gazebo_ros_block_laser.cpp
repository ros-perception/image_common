/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Ros Block Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id: gazebo_ros_block_laser.cc 6683 2008-06-25 19:12:30Z natepak $
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_block_laser.h>

#include "physics/World.hh"
#include "physics/HingeJoint.hh"
#include "sensors/Sensor.hh"
#include "sdf/interface/SDF.hh"
#include "sdf/interface/Param.hh"
#include "common/Exception.hh"
#include "sensors/RaySensor.hh"
#include "sensors/SensorTypes.hh"
#include "transport/Node.hh"

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include "tf/tf.h"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosBlockLaser::GazeboRosBlockLaser()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBlockLaser::~GazeboRosBlockLaser()
{
  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller / Custom Callback Queue
  this->laser_queue_.clear();
  this->laser_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_laser_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBlockLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, _sdf);

  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;

  // Get the world name.
  std::string worldName = _parent->GetWorldName();
  this->world_ = physics::get_world(worldName);

  last_update_time_ = this->world_->GetSimTime();

  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init(worldName);

  this->parent_ray_sensor_ = boost::shared_dynamic_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosBlockLaser controller requires a Ray Sensor as its parent");

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("Block laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->GetValueString();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("Block laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("Block laser plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->GetValueDouble();

  if (!_sdf->HasElement("hokuyoMinIntensity"))
  {
    ROS_INFO("Block laser plugin missing <hokuyoMinIntensity>, defaults to 101");
    this->hokuyo_min_intensity_ = 101;
  }
  else
    this->hokuyo_min_intensity_ = _sdf->GetElement("hokuyoMinIntensity")->GetValueDouble();

  ROS_INFO("INFO: gazebo_ros_laser plugin should set minimum intensity to %f due to cutoff in hokuyo filters." , this->hokuyo_min_intensity_);

  if (!_sdf->GetElement("updateRate"))
  {
    ROS_INFO("Block laser plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();
  // FIXME:  update the update_rate_


  this->laser_connect_count_ = 0;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  // set size of cloud message, starts at 0!! FIXME: not necessary
  this->cloud_msg_.points.clear();
  this->cloud_msg_.channels.clear();
  this->cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

  if (this->topic_name_ != "")
  {
    // Custom Callback Queue
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud>(
      this->topic_name_,1,
      boost::bind( &GazeboRosBlockLaser::LaserConnect,this),
      boost::bind( &GazeboRosBlockLaser::LaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }


  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  // start custom queue for laser
  this->callback_laser_queue_thread_ = boost::thread( boost::bind( &GazeboRosBlockLaser::LaserQueueThread,this ) );

}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosBlockLaser::LaserConnect()
{
  this->laser_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosBlockLaser::LaserDisconnect()
{
  this->laser_connect_count_--;

  if (this->laser_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBlockLaser::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
    common::Time sensor_update_time = this->parent_sensor_->GetLastUpdateTime();
    if (last_update_time_ < sensor_update_time)
    {
      this->PutLaserData(sensor_update_time);
      last_update_time_ = sensor_update_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_block_laser topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosBlockLaser::PutLaserData(common::Time &_updateTime)
{
  int i, hja, hjb;
  int j, vja, vjb;
  double vb, hb;
  int    j1, j2, j3, j4; // four corners indices
  double r1, r2, r3, r4, r; // four corner values + interpolated range
  double intensity;

  this->parent_ray_sensor_->SetActive(false);

  math::Angle maxAngle = this->parent_ray_sensor_->GetAngleMax();
  math::Angle minAngle = this->parent_ray_sensor_->GetAngleMin();

  double maxRange = this->parent_ray_sensor_->GetRangeMax();
  double minRange = this->parent_ray_sensor_->GetRangeMin();
  int rayCount = this->parent_ray_sensor_->GetRayCount();
  int rangeCount = this->parent_ray_sensor_->GetRangeCount();

  int verticalRayCount = this->parent_ray_sensor_->GetVerticalRayCount();
  int verticalRangeCount = this->parent_ray_sensor_->GetVerticalRangeCount();
  math::Angle verticalMaxAngle = this->parent_ray_sensor_->GetVerticalAngleMax();
  math::Angle verticalMinAngle = this->parent_ray_sensor_->GetVerticalAngleMin();

  double yDiff = maxAngle.Radian() - minAngle.Radian();
  double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();


  // set size of cloud message everytime!
  //int r_size = rangeCount * verticalRangeCount;
  this->cloud_msg_.points.clear();
  this->cloud_msg_.channels.clear();
  this->cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

  /***************************************************************/
  /*                                                             */
  /*  point scan from laser                                      */
  /*                                                             */
  /***************************************************************/
  boost::mutex::scoped_lock sclock(this->lock);
  // Add Frame Name
  this->cloud_msg_.header.frame_id = this->frame_name_;
  this->cloud_msg_.header.stamp.sec = _updateTime.sec;
  this->cloud_msg_.header.stamp.nsec = _updateTime.nsec;

  for (j = 0; j<verticalRangeCount; j++)
  {
    // interpolating in vertical direction
    vb = (verticalRangeCount == 1) ? 0 : (double) j * (verticalRayCount - 1) / (verticalRangeCount - 1);
    vja = (int) floor(vb);
    vjb = std::min(vja + 1, verticalRayCount - 1);
    vb = vb - floor(vb); // fraction from min

    assert(vja >= 0 && vja < verticalRayCount);
    assert(vjb >= 0 && vjb < verticalRayCount);

    for (i = 0; i<rangeCount; i++)
    {
      // Interpolate the range readings from the rays in horizontal direction
      hb = (rangeCount == 1)? 0 : (double) i * (rayCount - 1) / (rangeCount - 1);
      hja = (int) floor(hb);
      hjb = std::min(hja + 1, rayCount - 1);
      hb = hb - floor(hb); // fraction from min

      assert(hja >= 0 && hja < rayCount);
      assert(hjb >= 0 && hjb < rayCount);

      // indices of 4 corners
      j1 = hja + vja * rayCount;
      j2 = hjb + vja * rayCount;
      j3 = hja + vjb * rayCount;
      j4 = hjb + vjb * rayCount;
      // range readings of 4 corners
      r1 = std::min(this->parent_ray_sensor_->GetLaserShape()->GetRange(j1) , maxRange-minRange);
      r2 = std::min(this->parent_ray_sensor_->GetLaserShape()->GetRange(j2) , maxRange-minRange);
      r3 = std::min(this->parent_ray_sensor_->GetLaserShape()->GetRange(j3) , maxRange-minRange);
      r4 = std::min(this->parent_ray_sensor_->GetLaserShape()->GetRange(j4) , maxRange-minRange);

      // Range is linear interpolation if values are close,
      // and min if they are very different
      r = (1-vb)*((1 - hb) * r1 + hb * r2)
         +   vb *((1 - hb) * r3 + hb * r4);

      // Intensity is averaged
      intensity = 0.25*(this->parent_ray_sensor_->GetLaserShape()->GetRetro(j1) +
                        this->parent_ray_sensor_->GetLaserShape()->GetRetro(j2) +
                        this->parent_ray_sensor_->GetLaserShape()->GetRetro(j3) +
                        this->parent_ray_sensor_->GetLaserShape()->GetRetro(j4));

      // std::cout << " block debug "
      //           << "  ij("<<i<<","<<j<<")"
      //           << "  j1234("<<j1<<","<<j2<<","<<j3<<","<<j4<<")"
      //           << "  r1234("<<r1<<","<<r2<<","<<r3<<","<<r4<<")"
      //           << std::endl;

      // get angles of ray to get xyz for point
      double yAngle = 0.5*(hja+hjb) * yDiff / (rayCount -1) + minAngle.Radian();
      double pAngle = 0.5*(vja+vjb) * pDiff / (verticalRayCount -1) + verticalMinAngle.Radian();

      /***************************************************************/
      /*                                                             */
      /*  point scan from laser                                      */
      /*                                                             */
      /***************************************************************/
      if (r == maxRange - minRange)
      {
        // no noise if at max range
        geometry_msgs::Point32 point;
        point.x = (r+minRange) * cos(pAngle)*cos(yAngle);
        point.y = -(r+minRange) * sin(yAngle);
        point.z = (r+minRange) * sin(pAngle)*cos(yAngle);

        //pAngle is rotated by yAngle:
        point.x = (r+minRange) * cos(pAngle) * cos(yAngle);
        point.y = -(r+minRange) * cos(pAngle) * sin(yAngle);
        point.z = (r+minRange) * sin(pAngle);

        this->cloud_msg_.points.push_back(point); 
      } 
      else 
      { 
        geometry_msgs::Point32 point;
        point.x = (r+minRange) * cos(pAngle)*cos(yAngle) + this->GaussianKernel(0,this->gaussian_noise_) ;
        point.y = -(r+minRange) * sin(yAngle) + this->GaussianKernel(0,this->gaussian_noise_) ;
        point.z = (r+minRange) * sin(pAngle)*cos(yAngle) + this->GaussianKernel(0,this->gaussian_noise_) ;
        //pAngle is rotated by yAngle:
        point.x = (r+minRange) * cos(pAngle) * cos(yAngle) + this->GaussianKernel(0,this->gaussian_noise_);
        point.y = -(r+minRange) * cos(pAngle) * sin(yAngle) + this->GaussianKernel(0,this->gaussian_noise_);
        point.z = (r+minRange) * sin(pAngle) + this->GaussianKernel(0,this->gaussian_noise_);
        this->cloud_msg_.points.push_back(point); 
      } // only 1 channel 

      this->cloud_msg_.channels[0].values.push_back(intensity + this->GaussianKernel(0,this->gaussian_noise_)) ;
    }
  }
  this->parent_ray_sensor_->SetActive(true);

  // send data out via ros message
  this->pub_.publish(this->cloud_msg_);



}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosBlockLaser::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosBlockLaser::LaserQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->laser_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosBlockLaser::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  this->sim_time_  = msgs::Convert( _msg->sim_time() );

  math::Pose pose;
  pose.pos.x = 0.5*sin(0.01*this->sim_time_.Double());
  gzdbg << "plugin simTime [" << this->sim_time_.Double() << "] update pose [" << pose.pos.x << "]\n";
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBlockLaser)

}
