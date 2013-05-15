
/***************************************************************************
 *  gazebo_plugin.cpp - Plugin to access Gazebo features
 *
 *  Created: Fri Aug 24 11:04:04 2012
 *  Author  Bastian Klingen
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

#include "node_thread.h"

// from Gazebo
#include <gazebo/transport/Transport.hh>
#include <plugins/gazebo/gazebo_plugin.h>

using namespace fawkes;

/** @class GazeboPlugin "gazebo_plugin.h"
 * Plugin to access Gazebo from Fawkes.
 * This plugin integrates Gazebo and provides access to the Gazebo context
 * to other plugins.
 * @author Bastian Klingen
 */

/** Constructor.
 * @param config Fawkes configuration
 */
GazeboPlugin::GazeboPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new GazeboNodeThread());
}


PLUGIN_DESCRIPTION("Provides access to Gazebo")
EXPORT_PLUGIN(GazeboPlugin)
