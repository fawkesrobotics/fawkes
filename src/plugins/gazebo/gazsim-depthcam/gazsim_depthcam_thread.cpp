/***************************************************************************
 *  gazsim_depthcam_plugin.cpp - Plugin simulates a Depthcam in Gazebo and
 *                             provides a point cloud
 *
 *  Created: Fri Feb 19 20:59:05 2016
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

#include "gazsim_depthcam_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <utils/math/angle.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class DepthcamSimThread "gazsim_depthcam_thread.h"
 * Thread simulates a number of depthcams in Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
DepthcamSimThread::DepthcamSimThread()
  : Thread("DepthcamSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

void DepthcamSimThread::init()
{
  logger->log_debug(name(), "Initializing Simulation of the Depthcam");
  topic_name_ = config->get_string("/gazsim/depthcam/topic");
  width_ = config->get_float("/gazsim/depthcam/width");
  height_ = config->get_float("/gazsim/depthcam/height");
  frame_ = config->get_string("/gazsim/depthcam/frame");
  pcl_id_ = config->get_string("/gazsim/depthcam/pointcloud-id");

  depthcam_sub_ = gazebo_world_node->Subscribe(topic_name_, &DepthcamSimThread::on_depthcam_data_msg, this);

  //create pointcloud:
  pcl_ = new pcl::PointCloud<pcl::PointXYZ>();
  pcl_->is_dense = false;
  pcl_->width    = width_;
  pcl_->height   = height_;
  pcl_->points.resize(width_ * height_);
  pcl_->header.frame_id = frame_;

  pcl_manager->add_pointcloud(pcl_id_.c_str(), pcl_);
}

void DepthcamSimThread::finalize()
{
  pcl_manager->remove_pointcloud(pcl_id_.c_str());
}

void DepthcamSimThread::loop()
{
  //The interesting stuff happens in the callback of the depthcam
}

void DepthcamSimThread::on_depthcam_data_msg(ConstPointCloudPtr &msg)
{  
  // logger->log_info(name(), "Got Point Cloud!");

  //only write when pcl is used
  // if (pcl_.use_count() > 1)
  // {
    fawkes::Time capture_time = clock->now();

    pcl::PointCloud<pcl::PointXYZ> &pcl = **pcl_;
    pcl.header.seq += 1;
    pcl_utils::set_time(pcl_, capture_time);

    //insert or update points in pointcloud
    unsigned int idx = 0;
    for (unsigned int h = 0; h < height_; ++h) {
      for (unsigned int w = 0; w < width_; ++w, ++idx) {
        // Fill in XYZ
        pcl.points[idx].x = msg->points(idx).z();
        pcl.points[idx].y = -msg->points(idx).x();
        pcl.points[idx].z = msg->points(idx).y();
      }
    }
  // }
}
