
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

/** @class SimRobotinoSensorInterface "sim_robotinosensorinterface.cpp"
 * Thread to simulate the RobotinoSensorInterface
 * @author Frederik Zwilling
 */

/** Constructor. */

SimRobotinoSensorInterface::SimRobotinoSensorInterface(gazebo::transport::PublisherPtr controlPublisher, fawkes::Logger *logger, fawkes::BlackBoard *blackboard,   gazebo::transport::NodePtr gazebonode)
{
  this->controlPub = controlPublisher;
  this->logger = logger;
  this->blackboard = blackboard;
  this->gazebonode = gazebonode;
}

void SimRobotinoSensorInterface::init()
{
  logger->log_debug(name, "Initializing Simulation of RobotinoSensorInterface");
    
  //Open interfaces
  sens_if_ = blackboard->open_for_writing<RobotinoSensorInterface>("Robotino");

  //suscribe for messages
  gyroSub = gazebonode->Subscribe(std::string("~/RobotinoSim/Gyro/"), &SimRobotinoSensorInterface::OnGyroMsg, this);
  
  if(controlPub->HasConnections())
  {
    //Hello message
    msgs::Header helloMessage;
    helloMessage.set_str_id("Sim thread of RobotinoSensorInterface active");
    controlPub->Publish(helloMessage);  
  }
}

void SimRobotinoSensorInterface::finalize()
{
  blackboard->close(sens_if_);
}

void SimRobotinoSensorInterface::loop()
{
}

void SimRobotinoSensorInterface::OnGyroMsg(ConstVector3dPtr &msg)
{
  float yaw = msg->z();
  sens_if_->set_gyro_available(true);
  sens_if_->set_gyro_angle(yaw);
  sens_if_->write();
}
