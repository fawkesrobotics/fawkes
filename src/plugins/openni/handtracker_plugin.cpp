
/***************************************************************************
 *  handtracker_plugin.cpp - Plugin track hands using OpenNI
 *
 *  Created: Thu Mar 31 16:39:14 2011 (RoboCup German Open 2011)
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

#include "handtracker_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin to track hands using OpenNI.
 * This plugin uses OpenNI to track hands and publishes the information
 * to the blackboard.
 * @author Tim Niemueller
 */
class OpenNiHandTrackerPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  OpenNiHandTrackerPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new OpenNiHandTrackerThread());
  }
};

PLUGIN_DESCRIPTION("Hand tracker using OpenNI")
EXPORT_PLUGIN(OpenNiHandTrackerPlugin)
