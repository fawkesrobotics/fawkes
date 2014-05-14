/***************************************************************************
 *  robot_state_publisher_plugin.cpp - Robot State Publisher Plugin
 *
 *  Created on Thu Aug 22 11:18:00 2013
 *  Copyright (C) 2013 by Till Hofmann, AllemaniACs RoboCup Team
 *
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

#include <core/plugin.h>

#include "robot_state_publisher_thread.h"

using namespace fawkes;

/** This plugin publishes the robot's transforms given a URDF
 * model and joint values for the robot's joints
 * @author Till Hofmann
 */

class RobotStatePublisherPlugin : public fawkes::Plugin
{
public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RobotStatePublisherPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new RobotStatePublisherThread());
  }
};

PLUGIN_DESCRIPTION("Publishes transforms given a robot model and joint values")
EXPORT_PLUGIN(RobotStatePublisherPlugin)
