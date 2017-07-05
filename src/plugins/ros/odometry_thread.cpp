/***************************************************************************
 *  odometry_thread.h - Thread to publish odometry to ROS
 *
 *  Created: Fri Jun 1 13:29:39 CEST
 *  Copyright  2012  Sebastian Reuter
 *             2017  Till Hofmann
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

#include "odometry_thread.h"
#include <interfaces/MotorInterface.h>
#include <nav_msgs/Odometry.h>
#include <tf/types.h>

using namespace fawkes;

/** @class ROSOdometryThread "odometry_thread.h"
 * Thread to publish odometry to ROS.
 * @author Sebastian Reuter
 */

/** Constructor. */
ROSOdometryThread::ROSOdometryThread()
  : Thread("OdometryThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP)
{
}

void
ROSOdometryThread::init()
{
  std::string motor_if_id = config->get_string("/ros/odometry/motor_interface_id");
  cfg_odom_frame_id_ = config->get_string("/ros/odometry/odom_frame_id");
  cfg_base_frame_id_ = config->get_string("/ros/odometry/base_frame_id");
  if (config->exists("/ros/odometry/odom/covariance")) {
    odom_covariance_.assign(0.);
    std::vector<float> cfg_odom_covariance =
        config->get_floats("/ros/odometry/odom/covariance");
    for ( uint i = 0;
          i < cfg_odom_covariance.size() && i < odom_covariance_.size();
          i++) {
      odom_covariance_[i] = cfg_odom_covariance.size();
    }
  } else {
    odom_covariance_ = {
        1e-3, 0.,   0.,   0.,   0.,  0.,
        0.,   1e-3, 0.,   0.,   0.,  0.,
        0.,   0.,   1e-3, 0.,   0.,  0.,
        0.,   0.,   0.,   1e-3, 0.,   0.,
        0.,   0.,   0.,   0.,   1e-3, 0.,
        0.,   0.,   0.,   0.,   0.,   1e-3
    };
  }
  motor_if_ = blackboard->open_for_reading<MotorInterface>(motor_if_id.c_str());
  pub_ = rosnode->advertise<nav_msgs::Odometry>("odom", 10);
}

void
ROSOdometryThread::finalize()
{
  blackboard->close(motor_if_);
  pub_.shutdown();
}


void
ROSOdometryThread::publish_odom()
{
  nav_msgs::Odometry odom;
  //Header
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = cfg_odom_frame_id_;
  //Position
  odom.pose.pose.position.x = (double)motor_if_->odometry_position_x();
  odom.pose.pose.position.y = (double) motor_if_->odometry_position_y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.covariance = odom_covariance_;
  fawkes::tf::Quaternion q(motor_if_->odometry_orientation(), 0, 0);
  geometry_msgs::Quaternion odom_quat;
  odom_quat.x = q.x();
  odom_quat.y = q.y();
  odom_quat.z = q.z();
  odom_quat.w = q.w();
  odom.pose.pose.orientation = odom_quat;
  //Motion
  odom.child_frame_id = cfg_base_frame_id_;
  odom.twist.twist.linear.x = (double)motor_if_->vx();
  odom.twist.twist.linear.y = (double)motor_if_->vy();
  odom.twist.twist.angular.z = (double)motor_if_->omega();
  odom.twist.covariance = odom_covariance_;
  pub_.publish(odom);
}

void  ROSOdometryThread::loop() {
  motor_if_->read();
  publish_odom();
}
