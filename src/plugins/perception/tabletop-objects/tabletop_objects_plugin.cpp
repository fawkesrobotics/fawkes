
/***************************************************************************
 *  tabletop_objects_plugin.cpp - Segment table using PCL
 *
 *  Created: Fri Nov 04 23:38:26 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include "tabletop_objects_thread.h"
#ifdef HAVE_VISUAL_DEBUGGING
#  include "visualization_thread.h"
#endif

using namespace fawkes;

/** Plugin to segment a tabletop via PCL.
 * @author Tim Niemueller
 */
class TabletopObjectsPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  TabletopObjectsPlugin(Configuration *config)
    : Plugin(config)
  {
    TabletopObjectsThread *tabobjthr = new TabletopObjectsThread();
    thread_list.push_back(tabobjthr);
#ifdef HAVE_VISUAL_DEBUGGING
    TabletopVisualizationThread *visthr = new TabletopVisualizationThread();
    tabobjthr->set_visualization_thread(visthr);
    thread_list.push_back(visthr);
#endif
  }
};

PLUGIN_DESCRIPTION("Detect objects on tabletop using PCL")
EXPORT_PLUGIN(TabletopObjectsPlugin)
