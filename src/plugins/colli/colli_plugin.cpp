
/***************************************************************************
 *  colli_plugin.cpp - Fawkes Colli Plugin
 *
 *  Created: Wed Oct 16 18:00:00 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#include "act_thread.h"
#include "colli_thread.h"
#ifdef HAVE_VISUAL_DEBUGGING
 #include "visualization_thread.h"
#endif

#include <core/plugin.h>

using namespace fawkes;

/** Local odometry based path planning with collision avoidance.
 * @author Bahram Maleki-Fard
 */
class ColliPlugin : public fawkes::Plugin
{
public:
  /** Constructor.
   * @param config The fawkes configuration
   */
  ColliPlugin(Configuration *config)
      : Plugin(config)
  {
    ColliThread* colli_thread = new ColliThread();
    thread_list.push_back(colli_thread);
    
    thread_list.push_back(new ColliActThread(colli_thread));

#ifdef HAVE_VISUAL_DEBUGGING
    ColliVisualizationThread* vis_thread = new ColliVisualizationThread();
    thread_list.push_back(vis_thread);
    colli_thread->set_vis_thread( vis_thread );
#endif

  }
};

PLUGIN_DESCRIPTION("Local locomotion path planning with collision avoidance")
EXPORT_PLUGIN(ColliPlugin)

