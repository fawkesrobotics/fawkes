/***************************************************************************
 *  sim_robotinosensorinterface.h - Simulates the RobotinoSensorInterface
 *
 *  Created: Mon Jun 17 15:20:28 2013
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

#ifndef __SIM_ROBOTINO_SENSOR_INTERFACE_H_
#define __SIM_ROBOTINO_SENSOR_INTERFACE_H_

#include "sim_interface.h"



namespace fawkes {
  class RobotinoSensorInterface;
}

class SimRobotinoSensorInterface: public SimInterface
{
 public:
SimRobotinoSensorInterface(gazebo::transport::PublisherPtr controlPublisher, fawkes::Logger *logger, fawkes::BlackBoard *blackboard,   gazebo::transport::NodePtr gazebonode)
  : SimInterface(controlPublisher, logger, blackboard, gazebonode, "SimRobotinoSensorInterface")
  {};

  virtual void init();
  virtual void loop();
  virtual void finalize();
 
 private:
  //Suscribers to recieve messages from gazebo
  gazebo::transport::SubscriberPtr gyroSub;

  //Handler functions for incoming messages
  void OnGyroMsg(ConstVector3dPtr &msg);

  //provided interfaces
  fawkes::RobotinoSensorInterface *sens_if_;
};

#endif
