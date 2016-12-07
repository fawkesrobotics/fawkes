
/***************************************************************************
 *  pddl-planner_plugin.cpp - pddl-planner
 *
 *  Created: Wed Dec  7 19:09:44 2016
 *  Copyright  2016  Frederik Zwilling
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

#include "pddl-planner_thread.h"

using namespace fawkes;

/** @class PddlPlannerPlugin "pddl-planner_plugin.cpp"
 * Starts a pddl planner and writes the resulting plan into the robot memory
 * @author Frederik Zwilling
 */
class PddlPlannerPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fakwes configuration
   */
  PddlPlannerPlugin(Configuration *config)
     : Plugin(config)
  {
     thread_list.push_back(new PddlPlannerThread());
  }
};

PLUGIN_DESCRIPTION("Starts a pddl planner and writes the resulting plan into the robot memory")
EXPORT_PLUGIN(PddlPlannerPlugin)