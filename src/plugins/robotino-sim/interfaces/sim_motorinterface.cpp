
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
  logger_->log_debug(name_, "Initializing Simulation of MotorInterface");
    
  //Open interfaces
  motor_if_ = blackboard_->open_for_writing<MotorInterface>("Robotino");

  //create publisher for messages
  motor_move_pub_ = gazebonode_->Advertise<msgs::Vector3d>("~/RobotinoSim/MotorMove/");

  //suscribe for messages
  pos_sub_ = gazebonode_->Subscribe(std::string("~/RobotinoSim/Gps/"), &SimMotorInterface::on_pos_msg, this);
    
  if(control_pub_->HasConnections())
  {
    //Hello message
    msgs::Header helloMessage;
    helloMessage.set_str_id("Sim thread of MotorInterface active");
    control_pub_->Publish(helloMessage);  
  }
}

void SimMotorInterface::finalize()
{
  blackboard_->close(motor_if_);
}

void SimMotorInterface::loop()
{
  //work off all messages passed to the interface
  process_messages();
}


void SimMotorInterface::process_messages()
{
  while(motor_move_pub_->HasConnections() && !motor_if_->msgq_empty())
  {
    if (MotorInterface::TransRotMessage *msg =
	motor_if_->msgq_first_safe(msg))
    {
      //send command only if changed
      if(changed(msg->vx(), vx_, 0.01) || changed(msg->vy(), vy_, 0.01) || changed(msg->omega(), vomega_, 0.01))
      {
	vx_ = msg->vx();
	vy_ = msg->vy();
	vomega_ = msg->omega();
	msgs::Vector3d motorMove;
	motorMove.set_x(vx_);
	motorMove.set_y(vy_);
	motorMove.set_z(vomega_);
	motor_move_pub_->Publish(motorMove);

	//update interface
	motor_if_->set_vx(vx_);
	motor_if_->set_vy(vy_);
	motor_if_->set_omega(vomega_);
	//update interface
	motor_if_->write();
      }    
    }
    else if (motor_if_->msgq_first_is<MotorInterface::ResetOdometryMessage>())
      {
        x_offset_ += x_;
        y_offset_ += y_;
        ori_offset_ += ori_;
	x_ = 0.0;
	y_ = 0.0;
	ori_ = 0.0;
      }
    motor_if_->msgq_pop();
  }
}

//what to do if a pos-msg from gazebo arrives
void SimMotorInterface::on_pos_msg(ConstPosePtr &msg)
{
  //logger_->log_debug(name_, "Got Position MSG from gazebo with ori: %f", msg->z());
  //read out values + substract offset
  float new_x = msg->position().x() - x_offset_;
  float new_y = msg->position().y() - y_offset_;
  float new_ori = msg->orientation().z() - ori_offset_;

  //estimate path-length
  float length_driven = sqrt((new_x-x_) * (new_x-x_) + (new_y-y_) * (new_y-y_));

  //update stored values
  x_ = new_x;
  y_ = new_y;
  ori_ = new_ori;
  path_length_ += length_driven;

  //update interface
  motor_if_->set_odometry_position_x(x_);
  motor_if_->set_odometry_position_y(y_);
  motor_if_->set_odometry_orientation(ori_);
  motor_if_->set_odometry_path_length(path_length_);

  motor_if_->write();

  //publish transform (otherwise the transform can not convert /base_link to /odom)
  fawkes::Time now(clock_);
  tf::Transform t(tf::Quaternion(tf::Vector3(0,0,1),ori_),
		  tf::Vector3(x_, y_, 0.0));

  tf_publisher_->send_transform(t, now, "/odom", "/base_link");
}

bool changed(float before, float after, float relativeThreashold)
{
  return(fabs((before-after)/before) > relativeThreashold);
}
