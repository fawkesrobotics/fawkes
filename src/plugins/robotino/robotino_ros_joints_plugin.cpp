
/***************************************************************************
 *  robotino_ros_joints_plugin.cpp - Plugin to export ROS joint info
 *
 *  Created: Fri Mar 30 10:52:22 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "ros_joints_thread.h"

using namespace fawkes;

/** Plugin to publish Robotino joint info via ROS.
 * @author Tim Niemueller
 */
class RobotinoRosJointsPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RobotinoRosJointsPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new RobotinoRosJointsThread());
  }
};

PLUGIN_DESCRIPTION("Publish Robotino joint info via ROS (for model)")
EXPORT_PLUGIN(RobotinoRosJointsPlugin)
