
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

  //read config values
  slippery_wheels_enabled_ = config_->get_bool("gazsim/robotino/motor/slippery-wheels-enabled");
  slippery_wheels_threshold_ = config_->get_float("gazsim/robotino/motor/slippery-wheels-threshold");
  moving_speed_factor_ = config_->get_float("gazsim/robotino/motor/moving-speed-factor");
  rotation_speed_factor_ = config_->get_float("gazsim/robotino/motor/rotation-speed-factor");
    
  //Open interfaces
  motor_if_ = blackboard_->open_for_writing<MotorInterface>("Robotino");
  switch_if_ = blackboard_->open_for_writing<fawkes::SwitchInterface>("Robotino Motor");

  //enable motor by default
  switch_if_->set_enabled(true);
  switch_if_->write();

  //create publisher for messages
  motor_move_pub_ = gazebonode_->Advertise<msgs::Vector3d>("~/RobotinoSim/MotorMove/");

  //make sure variables are 0
  x_ = 0.0;
  y_ = 0.0;
  ori_ = 0.0;
  vx_ = 0.0;
  vy_ = 0.0;
  vomega_ = 0.0;
  x_offset_ = 0.0;
  y_offset_ = 0.0;
  ori_offset_ = 0.0;
  path_length_ = 0.0;
	
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
  blackboard_->close(switch_if_);
}

void SimMotorInterface::loop()
{
  //work off all messages passed to the interfaces
  process_messages();
}

void SimMotorInterface::send_transroot(double vx, double vy, double omega)
{
  msgs::Vector3d motorMove;
  motorMove.set_x(vx_);
  motorMove.set_y(vy_);
  motorMove.set_z(vomega_);
  motor_move_pub_->Publish(motorMove);
}

void SimMotorInterface::process_messages()
{
  //check messages of the switch interface
  while (!switch_if_->msgq_empty()) {
    if (SwitchInterface::DisableSwitchMessage *msg =
	switch_if_->msgq_first_safe(msg)) {
      switch_if_->set_enabled(false);
      //pause movement
      send_transroot(0, 0, 0);
    } else if (SwitchInterface::EnableSwitchMessage *msg =
	       switch_if_->msgq_first_safe(msg)) {
      switch_if_->set_enabled(true);
      //unpause movement
      send_transroot(vx_, vy_, vomega_);
    }
    switch_if_->msgq_pop();
    switch_if_->write();
  }

  //do not do anything if the motor is disabled
  if(!switch_if_->is_enabled())
  {
    return;
  }

  //check messages of the motor interface
  while(motor_move_pub_->HasConnections() && !motor_if_->msgq_empty())
  {
    if (MotorInterface::TransRotMessage *msg =
	motor_if_->msgq_first_safe(msg))
    {
      //send command only if changed
      if(changed(msg->vx(), vx_, 0.01) || changed(msg->vy(), vy_, 0.01) || changed(msg->omega(), vomega_, 0.01))
      {
	vx_ = msg->vx() * moving_speed_factor_;
	vy_ = msg->vy() * moving_speed_factor_;
	vomega_ = msg->omega() * rotation_speed_factor_;
	
	//send message to gazebo
	send_transroot(vx_, vy_, vomega_);

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
	last_vel_set_time_ = clock_->now();
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
  //calculate ori from quaternion
  float new_ori = tf::get_yaw(tf::Quaternion(msg->orientation().x(), msg->orientation().y()
					  , msg->orientation().z(), msg->orientation().w()));
  new_ori -= ori_offset_;
  
  //estimate path-length
  float length_driven = sqrt((new_x-x_) * (new_x-x_) + (new_y-y_) * (new_y-y_));
  
  if(slippery_wheels_enabled_)
  {
    //simulate slipping wheels when driving against an obstacle
    fawkes::Time new_time = clock_->now();
    double duration = new_time.in_sec() - last_pos_time_.in_sec();
    //calculate duration since the velocity was last set to filter slipping while accelerating
    double velocity_set_duration = new_time.in_sec() - last_vel_set_time_.in_sec();

    last_pos_time_ = new_time;
    

    double total_speed = sqrt(vx_ * vx_ + vy_ * vy_);
    if(length_driven < total_speed * duration * slippery_wheels_threshold_ && velocity_set_duration > duration)
    {
      double speed_abs_x = vx_ * cos(ori_) - vy_ * sin(ori_);
      double speed_abs_y = vx_ * sin(ori_) + vy_ * cos(ori_);
      double slipped_x = speed_abs_x * duration * slippery_wheels_threshold_;
      double slipped_y = speed_abs_y * duration * slippery_wheels_threshold_;
      logger_->log_info(name_, "Wheels are slipping (%f, %f)", slipped_x, slipped_y);
      new_x = x_ + slipped_x;
      new_y = y_ + slipped_y;
      //update the offset (otherwise the slippery error would be corrected in the next iteration)
      x_offset_ -= slipped_x;
      y_offset_ -= slipped_y;	      

      length_driven = sqrt((new_x-x_) * (new_x-x_) + (new_y-y_) * (new_y-y_));
    }

    // removed because of weird amcl results when driving as usual
    // //simulate slipping wheels when turning against an obstacle
    // if(abs(new_ori - ori_) < vomega_ * duration * slippery_wheels_threshold_ * 0.5)
    // {
    //   double slipped_ori = vomega_ * duration * slippery_wheels_threshold_ * 0.5;
    //   logger_->log_warn(name_, "Its is slippery here (ori)!!! %f", slipped_ori);
    //   new_ori = ori_ + slipped_ori;
    //   //update the offset (otherwise the slippery error would be corrected in the next iteration)
    //   ori_offset_ -= slipped_ori;	
    // }
  }

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
  return(before == 0.0 || after == 0.0 || fabs((before-after)/before) > relativeThreashold);
}
