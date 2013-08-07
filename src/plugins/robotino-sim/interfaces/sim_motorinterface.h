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

#include "sim_interface.h"
#include <tf/transform_publisher.h>
#include <utils/time/clock.h>



namespace fawkes {
  class MotorInterface;
}

class SimMotorInterface: public SimInterface
{
 public:
 SimMotorInterface(gazebo::transport::PublisherPtr controlPublisher, fawkes::Logger *logger, fawkes::BlackBoard *blackboard, gazebo::transport::NodePtr gazebonode, fawkes::Clock *clock, fawkes::tf::TransformPublisher *tf_publisher)
   : SimInterface(controlPublisher, logger, blackboard, gazebonode, "SimMotorInterface")
  {
    this->clock = clock;
    this->tf_publisher = tf_publisher;

    xOffset = 0.0;
    yOffset = 0.0;
    oriOffset = 0.0;
  };
  ~SimMotorInterface() {};

  virtual void init();
  virtual void loop();
  virtual void finalize();
 

 private:
  //Publisher to send messages to gazebo
  gazebo::transport::PublisherPtr motorMovePub;

  //Suscribers to recieve the robot's position from gazebo for odometry
  gazebo::transport::SubscriberPtr posSub;

  //provided interfaces
  fawkes::MotorInterface *motor_if_;
  
  //Needed for publishing the /base_link /odom transform
  fawkes::Clock *clock;
  fawkes::tf::TransformPublisher *tf_publisher;

  //Helper variables:
  //motorMovements last sent to gazebo
  float vx, vy, vomega, x, y, ori, pathLength;

  //Odometry offset
  float xOffset, yOffset, oriOffset;

  //Helper functions:
  void workOffMessages();

  //Handler functions for incoming messages
  void OnPosMsg(ConstVector3dPtr &msg);
};

#endif
