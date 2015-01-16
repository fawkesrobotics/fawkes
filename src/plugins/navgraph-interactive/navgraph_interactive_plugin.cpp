
/***************************************************************************
 *  navgraph_interactive_plugin.cpp - Interactive navgraph editing
 *
 *  Created: Thu Jan 15 16:23:34 2015
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

#include "navgraph_interactive_thread.h"

using namespace fawkes;

/** Interactive navgraph editing plugin.
 * @author Tim Niemueller
 */
class NavGraphInteractivePlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  NavGraphInteractivePlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new NavGraphInteractiveThread());
  }
};

PLUGIN_DESCRIPTION("Interactive navgraph editing via rviz")
EXPORT_PLUGIN(NavGraphInteractivePlugin)
