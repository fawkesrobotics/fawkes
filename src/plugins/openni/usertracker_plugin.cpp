
/***************************************************************************
 *  usertracker_plugin.cpp - Plugin track users using OpenNI
 *
 *  Created: Fri Mar 04 11:10:56 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "usertracker_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin to track users using OpenNI.
 * This plugin uses OpenNI to track users and publishes the information
 * to the blackboard.
 * @author Tim Niemueller
 */
class OpenNiUserTrackerPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  OpenNiUserTrackerPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new OpenNiUserTrackerThread());
  }
};

PLUGIN_DESCRIPTION("User tracker using OpenNI")
EXPORT_PLUGIN(OpenNiUserTrackerPlugin)
