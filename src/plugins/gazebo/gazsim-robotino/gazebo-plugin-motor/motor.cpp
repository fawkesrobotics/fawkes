/***************************************************************************
 *  motor.cpp - Plugin for controling a model through a simulated motor
 *
 *  Created: Wed Jan 29 16:10:17 2014
 *  Copyright  2014  Frederik Zwilling
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

#include <math.h>

#include <utils/misc/gazebo_api_wrappers.h>

#include "motor.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Motor)

Motor::Motor()
{
}

Motor::~Motor()
{
  printf("Destructing Motor Plugin!\n");
}

/** on loading of the plugin
 * @param _parent Parent Model
 */
void Motor::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Motor Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Motor::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the model name!
  this->node_->Init(model_->GetWorld()->GZWRAP_NAME()+"/"+name_);


  //initialize movement commands:
  vx_ = 0.0;
  vy_ = 0.0;
  vomega_ = 0.0;

  //create subscriber
  this->motor_move_sub_ = this->node_->Subscribe(std::string("~/RobotinoSim/MotorMove/"), &Motor::on_motor_move_msg, this);
}

/** Called by the world update start event
 */
void Motor::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //Apply movement command
  float x,y;
  float yaw = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;
  //foward part
  x = cos(yaw) * vx_;
  y = sin(yaw) * vx_;
  //sideways part
  x += cos(yaw + 3.1415926f / 2) * vy_;
  y += sin(yaw + 3.1415926f / 2) * vy_;
  // Apply velocity to the model.
  this->model_->SetLinearVel(gzwrap::Vector3d(x, y, 0));
  this->model_->SetAngularVel(gzwrap::Vector3d(0, 0, vomega_));
}

/** on Gazebo reset
 */
void Motor::Reset()
{
}

/** Functions for recieving Messages (registerd via suscribers)
 * @param msg message
 */ 
void Motor::on_motor_move_msg(ConstVector3dPtr &msg)
{
  //printf("Got MotorMove Msg!!! %f %f %f\n", msg->x(), msg->y(), msg->z());
  //Transform relative motion into ablosulte motion
  vx_ = msg->x();
  vy_ = msg->y();
  vomega_ = msg->z();
}
