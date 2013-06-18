
/***************************************************************************
 *  robotino-sim_thread.cpp - Thread simulate the Robotino in Gazebo by sending needed informations to the Robotino-plugin in Gazebo and recieving sensordata from Gazebo
 *
 *  Created: Fr 3. Mai 21:27:06 CEST 2013
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

#include "robotino_sim_thread.h"

#include <tf/types.h>
#include <stdio.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>


using namespace fawkes;
using namespace gazebo;

/** @class RobotinoSimThread "robotino-sim_thread.h"
 * Thread simulate the Robotino in Gazebo 
 * by sending needed informations to the Robotino-plugin in Gazebo
 * and recieving sensordata from Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
RobotinoSimThread::RobotinoSimThread()
  : Thread("RobotinoSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

void
RobotinoSimThread::init()
{
  //get a connection to gazebo (copied from gazeboscene)
  logger->log_debug(name(), "Creating Gazebo publishers");
  stringPub = gazebonode->Advertise<msgs::Header>("~/RobotinoSim/String/");

  if(stringPub->HasConnections())
  {
    //Hello world message
    msgs::Header helloMessage;
    helloMessage.set_str_id("Hello Gazebo-World!!!");
    stringPub->Publish(helloMessage);

  }

  //Loading interfaces to simulate
  if_robotinoSensorInterface = new SimRobotinoSensorInterface(stringPub, logger, blackboard, gazebonode);
  if_motorInterface = new SimMotorInterface(stringPub, logger, blackboard, gazebonode);
  if_robotinoSensorInterface->init();
  if_motorInterface->init();

}

void
RobotinoSimThread::finalize()
{
  //finalize simulated interfaces
  if_robotinoSensorInterface->finalize();
  if_motorInterface->finalize();

  //reset?
  stringPub.reset();
}

void
RobotinoSimThread::loop()
{
  if_robotinoSensorInterface->loop();
  if_motorInterface->loop();
}
