/***************************************************************************
 *  gps.cpp - Provides ground Truth position
 *
 *  Created: Tue Feb 04 15:06:06 2014
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

#include "gps.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Gps)

Gps::Gps()
{
}

Gps::~Gps()
{
  printf("Destructing Gps Plugin!\n");
}

void Gps::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Gps Plugin of model %s\n", name_.c_str());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Gps::OnUpdate, this, _1));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the model name!
  this->node_->Init(name_);

  //init last sent time
  last_sent_time_ = model_->GetWorld()->GetSimTime().Double();

  //create publisher
  this->gps_pub_ = this->node_->Advertise<msgs::Pose>("~/RobotinoSim/Gps/");
}

// Called by the world update start event
void Gps::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  //Send position information to Fawkes
  double time = model_->GetWorld()->GetSimTime().Double();
  if(time - last_sent_time_ > (1.0 / 5.0))
  {
    last_sent_time_ = time;
    send_position();
  }
}

void Gps::Reset()
{
}

void Gps::send_position()
{
  if(gps_pub_->HasConnections())
  {
    //build message
    msgs::Pose posMsg;
    posMsg.mutable_position()->set_x(this->model_->GetWorldPose().pos.x);
    posMsg.mutable_position()->set_y(this->model_->GetWorldPose().pos.y);
    posMsg.mutable_position()->set_z(this->model_->GetWorldPose().pos.z);
    posMsg.mutable_orientation()->set_x(this->model_->GetWorldPose().rot.x);
    posMsg.mutable_orientation()->set_y(this->model_->GetWorldPose().rot.y);
    posMsg.mutable_orientation()->set_z(this->model_->GetWorldPose().rot.z);
    posMsg.mutable_orientation()->set_w(this->model_->GetWorldPose().rot.w);

    //send
    gps_pub_->Publish(posMsg);
  }
}
