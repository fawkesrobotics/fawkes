
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

#include <utils/time/time.h>

#include <set>

#include <cstring>
#include <cerrno>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

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
  std::string replay_prefix = "/fawkes/bblogreplay/";

  std::string scenario = "";
  try {
    scenario = config->get_string((prefix + "scenario").c_str());
  } catch (Exception &e) {
    e.append("No scenario defined, configure %sscenario", prefix.c_str());
    throw;
  }

  /*
  bool generate_replay_config = false;
  try {
    generate_replay_config = config->get_bool((prefix + "generate_replay_config").c_str());
  } catch (Exception &e) {} // ignored, use default set above
  */

  std::string scenario_prefix = prefix + scenario + "/";
  std::string ifaces_prefix   = scenario_prefix + "interfaces/";

  std::string logdir = LOGDIR;
  bool        buffering = true;
  bool        flushing = false;
  try {
    logdir = config->get_string((scenario_prefix + "logdir").c_str());
  } catch (Exception &e) { /* ignored, use default set above */ }
  try {
    buffering = config->get_bool((scenario_prefix + "buffering").c_str());
  } catch (Exception &e) { /* ignored, use default set above */ }
  try {
    flushing = config->get_bool((scenario_prefix + "flushing").c_str());
  } catch (Exception &e) { /* ignored, use default set above */ }

  struct stat s;
  int err = stat(logdir.c_str(), &s);
  if (err != 0) {
    char buf[1024];
    Exception se ("Cannot access logdir %s (%s)",
		  logdir.c_str(), strerror_r(errno, buf, 1024));
    if (mkdir(logdir.c_str(), 0755) != 0) {
      se.append("Failed to create log directory (%s)",
		strerror_r(errno, buf, 1024));
      throw se;
    }
  } else if ( ! S_ISDIR(s.st_mode) ) {
    throw Exception("Logdir path %s is not a directory", logdir.c_str());
  }

  // We do not have the framework clock available at this point, but for the start
  // time of the log we are only interested in the system time anyway
  Time start;

  char date[21];
  Time now;
  struct tm *tmp = localtime(&(now.get_timeval()->tv_sec));
  strftime(date, 21, "%F-%H-%M-%S", tmp);
  std::string replay_cfg_prefix = replay_prefix + scenario + "-" + date + "/logs/";

  Configuration::ValueIterator *i = config->search(ifaces_prefix.c_str());
  while (i->next()) {
    std::string iface_name = std::string(i->path()).substr(ifaces_prefix.length());
    iface_name = iface_name.substr(0, iface_name.find("/"));

    //printf("Adding sync thread for peer %s\n", peer.c_str());
    BBLoggerThread *log_thread = new BBLoggerThread(i->get_string().c_str(),
						    logdir.c_str(),
						    buffering, flushing,
						    scenario.c_str(), &start);

    std::string filename = log_thread->get_filename();
    config->set_string((replay_cfg_prefix + iface_name + "/file").c_str(), filename);

    thread_list.push_back(log_thread);
  }
  delete i;

  if ( thread_list.empty() ) {
    throw Exception("No interfaces configured for logging, aborting");
  }

  BBLoggerThread *bblt = dynamic_cast<BBLoggerThread *>(thread_list.front());
  bblt->set_threadlist(thread_list);
}

PLUGIN_DESCRIPTION("Write BlackBoard interface data to files")
EXPORT_PLUGIN(BlackBoardLoggerPlugin)
