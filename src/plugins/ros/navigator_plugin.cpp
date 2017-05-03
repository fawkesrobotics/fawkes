
/***************************************************************************
 *  navigator_plugin.cpp - Robotino ROS Navigator Plugin
 *
 *  Created: Sat June 09 15:13:27 2012
 *  Copyright  2012  Sebastian Reuter
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
#include "navigator_thread.h"

using namespace fawkes;

/** Send locomotion commands to ROS.
 * @author Sebastian Reuter
 */
class RosNavigatorPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RosNavigatorPlugin(Configuration *config)
    : Plugin(config)
  {
    std::string prefix = "/ros/navigator";
    std::string cfg_prefix = prefix + "/";

    thread_list.push_back(new RosNavigatorThread(cfg_prefix));
  }
};

PLUGIN_DESCRIPTION("Send locomotion commands to ROS")
EXPORT_PLUGIN(RosNavigatorPlugin)
