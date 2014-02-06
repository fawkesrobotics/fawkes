/***************************************************************************
 *  gyro.cpp - Plugin for a gyro sensor on a model
 *
 *  Created: Tue Feb 04 14:43:59 2014
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

#include "gyro.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Gyro)

Gyro::Gyro()
{
}

Gyro::~Gyro()
{
  printf("Destructing Gyro Plugin!\n");
}

void Gyro::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Gyro Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Gyro::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the model name!
  this->node_->Init(name_);


  //create publisher
  this->gyro_pub_ = this->node_->Advertise<msgs::Vector3d>("~/RobotinoSim/Gyro/");
}

// Called by the world update start event
void Gyro::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //Send gyro information to Fawkes
  send_gyro();
}

void Gyro::Reset()
{
}

void Gyro::send_gyro()
{
  if(gyro_pub_->HasConnections())
  {
    //Read gyro from simulation
    float roll = this->model_->GetWorldPose().rot.GetAsEuler().x;
    float pitch = this->model_->GetWorldPose().rot.GetAsEuler().y;
    float yaw = this->model_->GetWorldPose().rot.GetAsEuler().z;

    //build message
    msgs::Vector3d gyroMsg;
    gyroMsg.set_x(roll);
    gyroMsg.set_y(pitch);
    gyroMsg.set_z(yaw);

    //send
    gyro_pub_->Publish(gyroMsg);
  }
}
