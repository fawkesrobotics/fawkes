
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
#include <math.h>
#include <utils/math/angle.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <interfaces/MotorInterface.h>
#include <tf/transform_publisher.h>
#include <utils/time/clock.h>


using namespace fawkes;
using namespace gazebo;

bool changed(float before, float after, float relativeThreashold);

void SimMotorInterface::init()
{
  logger->log_debug(name, "Initializing Simulation of MotorInterface");
    
  //Open interfaces
  motor_if_ = blackboard->open_for_writing<MotorInterface>("Robotino");

  //create publisher for messages
  motorMovePub = gazebonode->Advertise<msgs::Vector3d>("~/RobotinoSim/MotorMove/");

  //suscribe for messages
  posSub = gazebonode->Subscribe(std::string("~/RobotinoSim/Gps/"), &SimMotorInterface::OnPosMsg, this);
    
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
  workOffMessages();
}


void SimMotorInterface::workOffMessages()
{
  if(motorMovePub->HasConnections() && !motor_if_->msgq_empty())
  {
    if (MotorInterface::TransRotMessage *msg =
	motor_if_->msgq_first_safe(msg))
    {
      //send command only if changed
      if(changed(msg->vx(), vx, 0.01) || changed(msg->vy(), vy, 0.01) || changed(msg->omega(), vomega, 0.01))
      {
	vx = msg->vx();
	vy = msg->vy();
	vomega = msg->omega();
	msgs::Vector3d motorMove;
	motorMove.set_x(vx);
	motorMove.set_y(vy);
	motorMove.set_z(vomega);
	motorMovePub->Publish(motorMove);

	//update interface
	motor_if_->set_vx(vx);
	motor_if_->set_vy(vy);
	motor_if_->set_omega(vomega);
	//update interface
	motor_if_->write();
      }    
    }
    else if (motor_if_->msgq_first_is<MotorInterface::ResetOdometryMessage>())
      {
        xOffset += x;
        yOffset += y;
        oriOffset += ori;
	x = 0.0;
	y = 0.0;
	ori = 0.0;
      }
    motor_if_->msgq_pop();
  }
}

//what to do if a pos-msg from gazebo arrives
void SimMotorInterface::OnPosMsg(ConstVector3dPtr &msg)
{
  logger->log_debug(name, "Got Position MSG from gazebo");
  //read out values + substract offset
  float newX = msg->x() - xOffset;
  float newY = msg->y() - yOffset;
  float newOri = msg->z() - oriOffset;

  //estimate path-length
  float lengthDriven = sqrt((newX-x) * (newX-x) + (newY-y) * (newY-y));

  //update stored values
  x = newX;
  y = newY;
  ori = newOri;
  pathLength += lengthDriven;

  //update interface
  motor_if_->set_odometry_position_x(x);
  motor_if_->set_odometry_position_y(y);
  motor_if_->set_odometry_orientation(ori);
  motor_if_->set_odometry_path_length(pathLength);

  motor_if_->write();

  //publish transform (otherwise the transform can not convert /base_link to /odom)
  fawkes::Time now(clock);
  tf::Transform t(tf::Quaternion(tf::Vector3(0,0,1),
				 deg2rad(ori)),
		  tf::Vector3(x / 1000.f,
			      y / 1000.f,
			      0));

  tf_publisher->send_transform(t, now, "/odom", "/base_link");
}

bool changed(float before, float after, float relativeThreashold)
{
  return(fabs((before-after)/before) > relativeThreashold);
}
