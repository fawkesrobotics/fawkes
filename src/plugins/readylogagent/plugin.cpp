
/***************************************************************************
 *  readylog_agent.cpp - Fawkes Readylog Agent Plugin
 *
 *  Created: Wed Jul 15 11:33:53 2009
 *  Copyright  2009  Daniel Beck
 *
 *  $Id: worldmodel_plugin.h 1462 2008-10-27 15:02:00Z tim $
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

#include "plugin.h"
#include "control_thread.h"
#include "eclipse_thread.h"

/** @class ReadylogAgentPlugin "plugin.h"
 * The Readylog agent plugin.
 * @author Daniel Beck
 */

using namespace fawkes;

/** Constructor.
 * @param config the configuration
 */
ReadylogAgentPlugin::ReadylogAgentPlugin( Configuration* config )
  : Plugin( config )
{
  EclipseAgentThread* eclipse_thread = new EclipseAgentThread();
  thread_list.push_back( eclipse_thread );
  thread_list.push_back( new AgentControlThread( eclipse_thread ) );
}

PLUGIN_DESCRIPTION( "Runs the Readylog agent" )
EXPORT_PLUGIN( ReadylogAgentPlugin )
