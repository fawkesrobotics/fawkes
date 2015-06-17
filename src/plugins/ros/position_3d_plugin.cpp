/***************************************************************************
 *  position_3d_plugin.cpp - Publish 3D Position to ROS
 *
 *  Created: Wed Jul 16 17:04:42 2014
 *  Copyright  2014  Till Hofmann
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

#include "position_3d_thread.h"

using namespace fawkes;

/** Plugin to publish 3D Positions to ROS
 * @author Till Hofmann
 */
class RosPosition3DPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RosPosition3DPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new RosPosition3DThread());
  }
};

PLUGIN_DESCRIPTION("Plugin to publish 3D Positions to ROS")
EXPORT_PLUGIN(RosPosition3DPlugin)
