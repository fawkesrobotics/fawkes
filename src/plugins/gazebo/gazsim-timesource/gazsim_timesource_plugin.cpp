/***************************************************************************
 *  gazsim_timesource_plugin.cpp - Plugin sets the fawkes time
 *                                 to the simulation time
 *
 *  Created: Sat Sep 21 20:45:39 2013
 *  Copyright  2013  Frederik Zwilling
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

#include "gazsim_timesource_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin sets the simulation-time in fawkes
 * @author Frederik Zwilling
 */
class GazsimTimesourcePlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  GazsimTimesourcePlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new GazsimTimesourceThread());
  }
};


PLUGIN_DESCRIPTION("Provides Gazebo simulation-time")
EXPORT_PLUGIN(GazsimTimesourcePlugin)
