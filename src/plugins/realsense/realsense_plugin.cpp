
/***************************************************************************
 *  realsense_plugin.cpp - realsense
 *
 *  Plugin created: Mon Jun 13 17:09:44 2016

 *  Copyright  2016  Johannes Rothe
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

#include "realsense_thread.h"

using namespace fawkes;

/** Driver for the Intel RealSense camera.
 * @author Johannes Rothe
 */
class RealsensePlugin : public fawkes::Plugin
{
 public:
  /** Constructor
   * @param config Fakwes configuration
   */
  RealsensePlugin(Configuration *config)
     : Plugin(config)
  {
     thread_list.push_back(new RealsenseThread());
  }
};

PLUGIN_DESCRIPTION("Driver for Intel RealSense camera")
EXPORT_PLUGIN(RealsensePlugin)
