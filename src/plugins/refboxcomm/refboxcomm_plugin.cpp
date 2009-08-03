
/***************************************************************************
 *  refboxcomm_plugin.cpp - Fawkes RefBox Communication Plugin
 *
 *  Created: Sun Apr 19 13:08:11 2009
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
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

#include "refboxcomm_plugin.h"
#include "comm_thread.h"

using namespace fawkes;

/** @class RefBoxCommPlugin "refboxcomm_plugin.h"
 * Referee Box Communication Plugin for robotic soccer.
 * This plugin communicates with a refbox used for robotic soccer.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
RefBoxCommPlugin::RefBoxCommPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new RefBoxCommThread());
}


PLUGIN_DESCRIPTION("RefBox Communication Plugin")
EXPORT_PLUGIN(RefBoxCommPlugin)
