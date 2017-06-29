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

#include <utils/misc/gazebo_api_wrappers.h>

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

/** on loading of the plugin
 * @param _parent Parent Model
 */
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
  this->node_->Init(model_->GetWorld()->GZWRAP_NAME()+"/"+name_);


  //create publisher
  this->gyro_pub_ = this->node_->Advertise<msgs::Vector3d>("~/RobotinoSim/Gyro/");

  //init last sent time
  last_sent_time_ = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
  this->send_interval_ = 0.05;
}

/** Called by the world update start event
 */
void Gyro::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //Send gyro information to Fawkes
  double time = model_->GetWorld()->GZWRAP_SIM_TIME().Double();
  if(time - last_sent_time_ > send_interval_)
  {
    last_sent_time_ = time;
    send_gyro();
  }
}

/** on Gazebo reset
 */
void Gyro::Reset()
{
}

void Gyro::send_gyro()
{
  if(gyro_pub_->HasConnections())
  {
    //Read gyro from simulation
    float roll = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_X;
    float pitch = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Y;
    float yaw = this->model_->GZWRAP_WORLD_POSE().GZWRAP_ROT_EULER_Z;

    //build message
    msgs::Vector3d gyroMsg;
    gyroMsg.set_x(roll);
    gyroMsg.set_y(pitch);
    gyroMsg.set_z(yaw);

    //send
    gyro_pub_->Publish(gyroMsg);
  }
}
