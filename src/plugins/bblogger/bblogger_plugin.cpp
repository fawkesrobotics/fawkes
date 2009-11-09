
/***************************************************************************
 *  bblogger_plugin.cpp - Fawkes BlackBoard Logger Plugin
 *
 *  Created: Sat Nov 07 23:21:36 2009
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

#include "bblogger_plugin.h"
#include "log_thread.h"

#include <set>

using namespace fawkes;

/** @class BlackBoardLoggerPlugin "bblogger_plugin.h"
 * BlackBoard logger plugin.
 * This plugin logs one or more (or even all) interfaces to data files
 * for later replay or analyzing.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
BlackBoardLoggerPlugin::BlackBoardLoggerPlugin(Configuration *config)
  : Plugin(config)
{
  std::set<std::string> ifaces;

  std::string prefix = "/fawkes/bblogger/";
  std::string ifaces_prefix = prefix + "interfaces/";

  std::string logdir = LOGDIR;
  try {
    logdir = config->get_string((prefix + "logdir").c_str());
  } catch (Exception &e) {
    // ignored, use default set above
  }

  Configuration::ValueIterator *i = config->search(ifaces_prefix.c_str());
  while (i->next()) {

    //printf("Adding sync thread for peer %s\n", peer.c_str());
    BBLoggerThread *log_thread = new BBLoggerThread(i->get_string().c_str(),
						    logdir.c_str());
    thread_list.push_back(log_thread);
  }
  delete i;

  if ( thread_list.empty() ) {
    throw Exception("No interfaces configured for logging, aborting");
  } else {
  }
}

PLUGIN_DESCRIPTION("Write BlackBoard interface data to files")
EXPORT_PLUGIN(BlackBoardLoggerPlugin)
