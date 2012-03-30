
/***************************************************************************
 *  ros_joints_thread.cpp - Publish Robotino joint info via ROS
 *
 *  Created: Fri Mar 30 11:08:03 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
 *
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

#include "ros_joints_thread.h"

#include <utils/time/time.h>
#include <interfaces/RobotinoSensorInterface.h>
#include <ros/node_handle.h>

using namespace fawkes;

/** @class RobotinoRosJointsThread "ir_pcl_thread.h"
 * Robotino IR distances as point cloud.
 * This thread integrates into the Fawkes main loop at the SENSOR_PROCESS
 * hook and converts sensor data to a pointcloud
 * @author Tim Niemueller
 */

/** Constructor. */
RobotinoRosJointsThread::RobotinoRosJointsThread()
  : Thread("RobotinoRosJointsThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE)
{
}


void
RobotinoRosJointsThread::init()
{
  sens_if_ = blackboard->open_for_reading<RobotinoSensorInterface>("Robotino");
  sens_if_->read();

  pub_joints_ = rosnode->advertise<sensor_msgs::JointState>("joint_states", 1);

  joint_state_msg_.name.resize(3);
  joint_state_msg_.position.resize(3, 0.0);
  joint_state_msg_.velocity.resize(3, 0.0);
  joint_state_msg_.name[0] = "wheel2_joint";
  joint_state_msg_.name[1] = "wheel0_joint";
  joint_state_msg_.name[2] = "wheel1_joint";

}


void
RobotinoRosJointsThread::finalize()
{
  blackboard->close(sens_if_);
  pub_joints_.shutdown();
}

void
RobotinoRosJointsThread::loop()
{
  // update sensor values in interface
  sens_if_->read();

  if (sens_if_->changed()) {
    const Time *ct = sens_if_->timestamp();
    float   *mot_velocity = sens_if_->mot_velocity();
    int32_t *mot_position = sens_if_->mot_position();

    joint_state_msg_.header.seq   += 1;
    joint_state_msg_.header.stamp  = ros::Time(ct->get_sec(), ct->get_usec() * 1e3);

    joint_state_msg_.velocity[0] = (mot_velocity[2] / 16) * (2 * M_PI) / 60;
    joint_state_msg_.velocity[1] = (mot_velocity[0] / 16) * (2 * M_PI) / 60;
    joint_state_msg_.velocity[2] = (mot_velocity[1] / 16) * (2 * M_PI) / 60;

    joint_state_msg_.position[0] = (mot_position[2] / 16) * (2 * M_PI);
    joint_state_msg_.position[1] = (mot_position[0] / 16) * (2 * M_PI);
    joint_state_msg_.position[2] = (mot_position[1] / 16) * (2 * M_PI);

    pub_joints_.publish(joint_state_msg_);
  }
}
