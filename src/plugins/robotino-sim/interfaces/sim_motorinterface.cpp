
/***************************************************************************
 *  sim_motorinterface.cpp - Simulates the MotorInterface
 *
 *  Created: Mon Jun 17 15:26:17 2013
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

#include "sim_motorinterface.h"

#include <tf/types.h>
#include <stdio.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <interfaces/MotorInterface.h>


using namespace fawkes;
using namespace gazebo;


void SimMotorInterface::init()
{
  logger->log_debug(name, "Initializing Simulation of MotorInterface");
    
  //Open interfaces
  motor_if_ = blackboard->open_for_writing<MotorInterface>("Robotino");

  //create publisher for messages
  motorMovePub = gazebonode->Advertise<msgs::Vector3d>("~/RobotinoSim/MotorMove/");
  
  if(controlPub->HasConnections())
  {
    //Hello message
    msgs::Header helloMessage;
    helloMessage.set_str_id("Sim thread of MotorInterface active");
    controlPub->Publish(helloMessage);  
  }
}

void SimMotorInterface::finalize()
{
  blackboard->close(motor_if_);
}

void SimMotorInterface::loop()
{
  sendMotorMove();
}


void SimMotorInterface::sendMotorMove()
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
