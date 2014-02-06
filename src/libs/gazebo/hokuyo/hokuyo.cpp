/***************************************************************************
 *  hokuyo.h - Plugin for getting laser sensor data from the simulation
 *
 *  Created: Wed Jan 29 17:04:23 2014
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
#include <gazebo/sensors/sensors.hh>
#include "hokuyo.h"

using namespace gazebo;

// Register this plugin to make it available in the simulator
GZ_REGISTER_MODEL_PLUGIN(Hokuyo)

Hokuyo::Hokuyo()
{
}

Hokuyo::~Hokuyo()
{
  printf("Destructing Hokuyo Plugin!\n");
}

void Hokuyo::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
{
  // Store the pointer to the model
  this->model_ = _parent;

  //get the model-name
  this->name_ = model_->GetName();
  printf("Loading Hokuyo Plugin of model %s\n", name_.c_str());

  std::string laser_name =  model_->GetWorld()->GetName() + "::" + name_ + "::hokuyo::link::laser";
  
  this->parent_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(sensors::get_sensor(laser_name.c_str()));

  //Create the communication Node for communication with fawkes
  this->node_ = transport::NodePtr(new transport::Node());
  //the namespace is set to the model name!
  this->node_->Init(name_);


  //Register OnNewLaserScans function
  this->new_laser_scans_connection_ = this->parent_sensor_->GetLaserShape()->ConnectNewLaserScans(boost::bind(&Hokuyo::on_new_laser_scans, this));

  //create publisher
  this->laser_pub_ = this->node_->Advertise<msgs::LaserScan>("~/RobotinoSim/LaserSensor/");
}

void Hokuyo::on_new_laser_scans()
{
  if(laser_pub_->HasConnections())
  {
    //Get relevant data
    int numRays = parent_sensor_->GetRangeCount();
    float angleMin = parent_sensor_->GetAngleMin().Radian();
    float angleMax = parent_sensor_->GetAngleMax().Radian();
    float angleStep = parent_sensor_->GetAngleResolution();
    float rangeMin = parent_sensor_->GetRangeMin();
    float rangeMax = parent_sensor_->GetRangeMax();
    

    //create Protobuf message
    msgs::LaserScan laserMsg;
    laserMsg.set_frame("/base_laser");
    laserMsg.set_angle_min(angleMin);
    laserMsg.set_angle_max(angleMax);
    laserMsg.set_angle_step(angleStep);
    laserMsg.set_range_min(rangeMin);
    laserMsg.set_range_max(rangeMax);
    for(int i = 0; i < numRays; i++)
    {
    
      laserMsg.add_ranges(parent_sensor_->GetRange(i));
      //laserMsg.add_intensities(-1);//I think I don't need the intensity
      //printf("Ray number %d range %f\n", i, parent_sensor_->GetRange(i));
    }
    //dummy for world pose
    laserMsg.mutable_world_pose()->mutable_position()->set_x(0);
    laserMsg.mutable_world_pose()->mutable_position()->set_y(0);
    laserMsg.mutable_world_pose()->mutable_position()->set_z(0);
    laserMsg.mutable_world_pose()->mutable_orientation()->set_x(0);
    laserMsg.mutable_world_pose()->mutable_orientation()->set_y(0);
    laserMsg.mutable_world_pose()->mutable_orientation()->set_z(0);
    laserMsg.mutable_world_pose()->mutable_orientation()->set_w(0);
 
    //send message
    laser_pub_->Publish(laserMsg);
    }
}


void Hokuyo::Reset()
{
}
