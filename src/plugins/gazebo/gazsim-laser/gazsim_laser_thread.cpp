
/***************************************************************************
 *  gazsim_laser_thread.cpp - Thread simulate the Hokuyo in Gazebo
 *
 *  Created: Thu Aug 08 15:51:41 2013
 *  Copyright  2013  Frederik Zwilling
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

#include "gazsim_laser_thread.h"

#include <tf/types.h>
#include <utils/math/angle.h>
#include <core/threading/mutex_locker.h>

#include <interfaces/Laser360Interface.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

#include <cstdio>
#include <cmath>

using namespace fawkes;
using namespace gazebo;

/** @class LaserSimThread "gazsim_laser_thread.h"
 * Thread simulates the Hokuyo in Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
LaserSimThread::LaserSimThread()
  : Thread("LaserSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

void LaserSimThread::init()
{
  logger->log_debug(name(), "Initializing Simulation of the Laser Sensor");

  //read config values
  max_range_ = config->get_float("/gazsim/laser/max_range");
  laser_topic_ = config->get_string("/gazsim/topics/laser");
  interface_id_ = config->get_string("/gazsim/laser/interface-id");
  frame_id_ = config->get_string("/gazsim/laser/frame-id");

  //open interface
  laser_if_ = blackboard->open_for_writing<Laser360Interface>(interface_id_.c_str());
  laser_if_->set_auto_timestamping(false);

  //subscribing to gazebo publisher
  laser_sub_ = gazebonode->Subscribe(laser_topic_, &LaserSimThread::on_laser_data_msg, this);

  //initialize laser data
  laser_data_ = (float *)malloc(sizeof(float) * 360);
  laser_time_ = new Time(clock);
  new_data_ = false;

  //set frame in the interface
  laser_if_->set_frame(frame_id_.c_str());
}

void LaserSimThread::finalize()
{
  blackboard->close(laser_if_);
  free(laser_data_);
  delete laser_time_;
}

void LaserSimThread::loop()
{
  if(new_data_)
  {
    //write interface
    laser_if_->set_distances(laser_data_);
    laser_if_->set_timestamp(laser_time_);
    laser_if_->write();

    new_data_ = false;
  }  
}

void LaserSimThread::on_laser_data_msg(ConstLaserScanStampedPtr &msg)
{
  //logger->log_info(name(), "Got new Laser data.\n");

  MutexLocker lock(loop_mutex);

  const gazebo::msgs::LaserScan &scan = msg->scan();

  //calculate start angle
  int start_index = (scan.angle_min() + 2* M_PI) / M_PI * 180;
  
  int number_beams = scan.ranges_size();

  *laser_time_ = clock->now();
  
  //copy laser data
  for(int i = 0; i < number_beams; i++)
  {
    const float range = scan.ranges(i);
    if(range < max_range_)
    {
      laser_data_[(start_index + i) % 360] = range; 
    }
    else
    {
      laser_data_[(start_index + i) % 360] = NAN;
    }

  }
  new_data_  = true;
}
