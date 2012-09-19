
/***************************************************************************
 *  navgraph_plugin.cpp - Graph-based global path planning
 *
 *  Created: Tue Sep 18 15:55:38 2012
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

#include "navgraph_thread.h"
#ifdef HAVE_VISUALIZATION
#  include "visualization_thread.h"
#endif

using namespace fawkes;

/** Graph-based global path planning.
 * @author Tim Niemueller
 */
class NavGraphPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  NavGraphPlugin(Configuration *config)
    : Plugin(config)
  {
#ifdef HAVE_VISUALIZATION
    bool use_vis = false;
    try {
      use_vis = config->get_bool("/plugins/navgraph/visualization");
    } catch (Exception &e) {} // ignored, use default
    if (use_vis) {
      NavGraphVisualizationThread *vt = new NavGraphVisualizationThread();
      thread_list.push_back(new NavGraphThread(vt));
      thread_list.push_back(vt);
    } else {
      thread_list.push_back(new NavGraphThread());
    }
#else
    thread_list.push_back(new NavGraphThread());
#endif
  }
};

PLUGIN_DESCRIPTION("Graph-based local path planning")
EXPORT_PLUGIN(NavGraphPlugin)
