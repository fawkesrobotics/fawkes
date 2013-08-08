
/***************************************************************************
 *  laser_sim_thread.cpp - Thread simulate the Hokuyo in Gazebo
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

#include "laser_sim_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <utils/math/angle.h>

#include <interfaces/Laser360Interface.h>
#include <tf/transform_publisher.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

using namespace fawkes;
using namespace gazebo;

/** @class LaserSimThread "laser_sim_thread.h"
 * Thread simulates the Hokuyo in Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
LaserSimThread::LaserSimThread()
  : Thread("LaserSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE),
    TransformAspect(TransformAspect::ONLY_PUBLISHER, "Laser Transform??????")
{
}

void LaserSimThread::init()
{
  logger->log_debug(name(), "Initializing Simulation of the Laser Sensor");

  //open interface
  laser_if_ = blackboard->open_for_writing<Laser360Interface>("Laser urg"); //not sure if the name is right

  //subscribing to gazebo publisher
  laser_sub_ = gazebonode->Subscribe(std::string("~/RobotinoSim/LaserSensor/"), &LaserSimThread::on_laser_data_msg, this);
}

void LaserSimThread::finalize()
{
  blackboard->close(laser_if_);
}

void LaserSimThread::loop()
{
}

void LaserSimThread::on_laser_data_msg(ConstLaserScanPtr &msg)
{
  logger->log_info(name(), "Got new Laser data.\n");
}
