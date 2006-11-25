
/***************************************************************************
 *  component_ids.h - Fawkes component IDs
 *
 *  Created: Tue Nov 21 18:18:58 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */


/* You need a component ID for each network handler. This is to be globally
 * unique throughout the whole software. In order to organize these IDs in a
 * central place this file has been created. Add an ID for each network handler
 * that you create.
 *
 * Note that component IDs 0 to 1000 are reserved for base system network
 * handlers and may not be used for plugin network handlers.
 *
 * All ID names must be prefixed with FAWKES_CID_. Please add a short comment
 * where the component is implemented in the tree.
 */

#ifndef __NETCOMM_FAWKES_COMPONENT_IDS_H_
#define __NETCOMM_FAWKES_COMPONENT_IDS_H_


/* **** System CIDs **** */

// PluginManager: mainapp/plugin_manager.h
#define FAWKES_CID_PLUGINMANAGER         1

// BlackBoard: blackboard/network_handler.h
#define FAWKES_CID_BLACKBOARD            2



/* **** Normal component CIDs **** */

#define FAWKES_CID_FIREVISION         1001


#endif
