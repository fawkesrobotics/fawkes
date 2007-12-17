
/***************************************************************************
 *  worldmodel_plugin.cpp - Fawkes WorldModel Plugin
 *
 *  Created: Fri Jun 29 11:47:53 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/worldmodel/worldmodel_plugin.h>
#include <plugins/worldmodel/thread.h>
#include <plugins/worldmodel/net_thread.h>

/** @class WorldModelPlugin plugins/worldmodel/worldmodel_plugin.h
 * Simple worldmodel plugin.
 * This plugin facilitates the Fawkes worldmodel, which agglomerates data produced
 * by low level Fawkes plugins and provides a consistent view of the world to
 * modules on higher levels.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
WorldModelPlugin::WorldModelPlugin()
  : Plugin("worldmodel")
{
  thread_list.push_back(new WorldModelThread());
  thread_list.push_back(new WorldModelNetworkThread());
}

EXPORT_PLUGIN(WorldModelPlugin)
