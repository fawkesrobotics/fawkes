
/***************************************************************************
 *  sim_robotinosensorinterface.cpp - Simulates the RobotinoSensorInterface
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

#include "sim_robotinosensorinterface.h"

#include <tf/types.h>
#include <stdio.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <interfaces/RobotinoSensorInterface.h>


using namespace fawkes;
using namespace gazebo;


void SimRobotinoSensorInterface::init()
{
  logger_->log_debug(name_, "Initializing Simulation of RobotinoSensorInterface");
    
  //Open interfaces
  sens_if_ = blackboard_->open_for_writing<RobotinoSensorInterface>("Robotino");

  //suscribe for messages
  gyro_sub_ = gazebonode_->Subscribe(std::string("~/RobotinoSim/Gyro/"), &SimRobotinoSensorInterface::on_gyro_msg, this);
  
  if(control_pub_->HasConnections())
  {
    //Hello message
    msgs::Header helloMessage;
    helloMessage.set_str_id("Sim thread of RobotinoSensorInterface active");
    control_pub_->Publish(helloMessage);  
  }
}

void SimRobotinoSensorInterface::finalize()
{
  blackboard_->close(sens_if_);
}

void SimRobotinoSensorInterface::loop()
{
}

void SimRobotinoSensorInterface::on_gyro_msg(ConstVector3dPtr &msg)
{
  float yaw = msg->z();
  sens_if_->set_gyro_available(true);
  sens_if_->set_gyro_angle(yaw);
  sens_if_->write();
}
