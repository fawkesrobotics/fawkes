/***************************************************************************
 *  sim_motorinterface.h - Simulates the Motorinterface
 *
 *  Created: Tue Jun 18 11:35:52 2013
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

#ifndef __SIM_MOTOR_INTERFACE_H_
#define __SIM_MOTOR_INTERFACE_H_

#include <logging/logger.h>
#include <blackboard/blackboard.h>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>

namespace fawkes {
  class MotorInterface;
}

class SimMotorInterface
{
 public:
SimMotorInterface(gazebo::transport::PublisherPtr controlPublisher, fawkes::Logger *logger, fawkes::BlackBoard *blackboard,   gazebo::transport::NodePtr gazebonode);

  void init();
  void loop();
  void finalize();
 
 private:
  //Name for the logger
  const char* name = "SimMotorInterface";
  fawkes::Logger *logger;
  fawkes::BlackBoard *blackboard;
  gazebo::transport::NodePtr gazebonode;

  //Publisher to send control-messages to gazebo
  gazebo::transport::PublisherPtr controlPub;
  gazebo::transport::PublisherPtr motorMovePub;

  //provided interfaces
  fawkes::MotorInterface *motor_if_;

  //Helper variables:
  //motorMovements last sent to gazebo
  float vx, vy, vomega;

  //Helper functions:
  void sendMotorMove();
};

#endif
