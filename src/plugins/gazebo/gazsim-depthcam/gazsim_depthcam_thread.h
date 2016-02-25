/***************************************************************************
 *  gazsim_depthcam_plugin.cpp - Plugin simulates a depthcam in Gazebo and
 *                             provides a point cloud
 *
 *  Created: Fri Feb 19 20:58:32 2016
 *  Copyright  2016  Frederik Zwilling
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_GAZSIM_DEPTHCAM_THREAD_H_
#define __PLUGINS_GAZSIM_DEPTHCAM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <string.h>

#include <aspect/pointcloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>

class DepthcamSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect,
  public fawkes::PointCloudAspect
{
 public:
  DepthcamSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  gazebo::transport::SubscriberPtr depthcam_sub_;

  //handler function for incoming depthcam data messages
  void on_depthcam_data_msg(ConstPointCloudPtr &msg);

  //config values
  //topic name of the gazebo message publisher
  std::string topic_name_;
  unsigned int width_;
  unsigned int height_;
  //id of the shared memory object
  std::string shm_id_;
  std::string frame_;
  std::string pcl_id_;

  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ> > pcl_;
};

#endif
