
/***************************************************************************
 *  clock_plugin.cpp - Plugin to publish clock to ROS
 *
 *  Created: Sun Jul 12 16:11:33 2015
 *  Copyright  2015  Tim Niemueller
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

#include "clock_thread.h"

using namespace fawkes;

/** Plugin to publish clock to ROS.
 * @author Tim Niemueller
 */
class RosClockPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RosClockPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new RosClockThread());
  }
};

PLUGIN_DESCRIPTION("Publish clock to ROS")
EXPORT_PLUGIN(RosClockPlugin)
