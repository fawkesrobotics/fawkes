
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

#include <interfaces/MotorInterface.h>
#include <interfaces/BatteryInterface.h>
#include <interfaces/RobotinoSensorInterface.h>


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
  //Open interfaces
  motor_if_ = blackboard->open_for_writing<MotorInterface>("Robotino");
  batt_if_ = blackboard->open_for_writing<BatteryInterface>("Robotino");
  sens_if_ = blackboard->open_for_writing<RobotinoSensorInterface>("Robotino");

  //get a connection to gazebo (copied from gazeboscene)
  logger->log_debug(name(), "Creating Gazebo publishers");
  stringPub = gazebonode->Advertise<msgs::Header>("~/RobotinoSim/String/");
  motorMovePub = gazebonode->Advertise<msgs::Vector3d>("~/RobotinoSim/MotorMove/");

  logger->log_debug(name(), "Try Suscribing");
  gyroSub = gazebonode->Subscribe(std::string("~/RobotinoSim/Gyro/"), &RobotinoSimThread::OnGyroMsg, this);

  //stringPub->WaitForConnection();
  logger->log_debug(name(), "Gazebo publishers created and connected");
  
  if(stringPub->HasConnections())
  {
    logger->log_info(name(), "Try sending messages");
    //Hello world message
    msgs::Header helloMessage;
    helloMessage.set_str_id("Hello Gazebo-World!!!");
    stringPub->Publish(helloMessage);
  }
}

void
RobotinoSimThread::finalize()
{
  blackboard->close(motor_if_);
  blackboard->close(batt_if_);
  blackboard->close(sens_if_);
  //reset?
  stringPub.reset();
  motorMovePub.reset();
}

void
RobotinoSimThread::loop()
{
  sendMotorMove();
}

void RobotinoSimThread::OnGyroMsg(ConstVector3dPtr &msg)
{
  float yaw = msg->z();
  logger->log_info(name(), "Got GyroMsg! Yaw=%f\n", yaw);
  sens_if_->set_gyro_available(true);
  sens_if_->set_gyro_angle(yaw);
  sens_if_->write();
}

void RobotinoSimThread::sendMotorMove()
{
  if(motorMovePub->HasConnections() && !motor_if_->msgq_empty())
  {
    if (MotorInterface::TransRotMessage *msg =
	motor_if_->msgq_first_safe(msg))
    {
      //send command only if changed
      //TODO: send if there is a new connection
      if(msg->vx() != vx || msg->vy() != vy || msg->omega() != vomega)
      {
	vx = msg->vx();
	vy = msg->vy();
	vomega = msg->omega();
	msgs::Vector3d motorMove;
	motorMove.set_x(vx);
	motorMove.set_y(vy);
	motorMove.set_z(vomega);
	motorMovePub->Publish(motorMove);
      }    
    }
    motor_if_->msgq_pop();
  }
}
