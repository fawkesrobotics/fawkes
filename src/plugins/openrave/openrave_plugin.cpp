
/***************************************************************************
 *  openrave_plugin.cpp - Fawkes OpenRAVE Plugin
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#include "openrave_plugin.h"

#include "openrave_thread.h"
#include "message_handler_thread.h"

using namespace fawkes;

/** @class OpenRavePlugin <plugins/openrave/openrave_plugin.h>
 * OpenRAVE Connector plugin.
 * This plugin provides access to OpenRAVE for other Fawkes plugins.
 * It builds up an environment, manages objects and robots in that environment
 * and handles ik solving and path-planning.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param config Fawkes configuration
 */
OpenRavePlugin::OpenRavePlugin(Configuration *config)
  : Plugin(config)
{
  OpenRaveThread* or_thread = new OpenRaveThread();

  thread_list.push_back(or_thread);
  thread_list.push_back(new OpenRaveMessageHandlerThread(or_thread));
}


PLUGIN_DESCRIPTION("OpenRAVE Connector Plugin")
EXPORT_PLUGIN(OpenRavePlugin)
