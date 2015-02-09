
/***************************************************************************
 *  navgraph_generator_plugin.cpp - Graph-based global path planning
 *
 *  Created: Mon Feb 09 17:34:21 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_generator_thread.h"

using namespace fawkes;

/** Plugin to generate navgraphs based on given parameters.
 * @author Tim Niemueller
 */
class NavGraphGeneratorPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  NavGraphGeneratorPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new NavGraphGeneratorThread());
  }
};

PLUGIN_DESCRIPTION("Plugin to instruct and perform navgraph generation")
EXPORT_PLUGIN(NavGraphGeneratorPlugin)
