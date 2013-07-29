
/***************************************************************************
 *  sim_motorinterface.cpp - Simulates an Interface
 *
 *  Created: Wed Jun 19 09:30:26 2013
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

#include "sim_interface.h"

#include <tf/types.h>
#include <stdio.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

/** @class SimInterface "sim_interface.cpp"
 * Class to simulate an Interface
 * @author Frederik Zwilling
 */

/** Constructor. */
/*
SimInterface::SimInterface(gazebo::transport::PublisherPtr controlPublisher, fawkes::Logger *logger, fawkes::BlackBoard *blackboard,   gazebo::transport::NodePtr gazebonode)
{
  this->controlPub = controlPublisher;
  this->logger = logger;
  this->blackboard = blackboard;
  this->gazebonode = gazebonode;
}
*/
