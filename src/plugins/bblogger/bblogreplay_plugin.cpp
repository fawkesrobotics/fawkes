
/***************************************************************************
 *  bblogreplay_plugin.cpp - Fawkes BlackBoard Log Replay Plugin
 *
 *  Created: Wed Feb 17 01:53:00 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *             2010  Masrur Doostdar <doostdar@kbsg.rwth-aachen.de>
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

#include "bblogreplay_plugin.h"
#include "logreplay_thread.h"
#include "logreplay_bt_thread.h"

#include <utils/time/time.h>

#include <set>
#include <memory>

#include <cstring>
#include <cerrno>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace fawkes;

/** @class BlackBoardLogReplayPlugin "bblogger_plugin.h"
 * BlackBoard log replay plugin.
 * This plugin replay one or more logfiles into interfaces of the local blackboard
 *
 * @author Masrur Doostdar
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
BlackBoardLogReplayPlugin::BlackBoardLogReplayPlugin(Configuration *config)
  : Plugin(config)
{
  std::set<std::string> logs;

  std::string prefix = "/fawkes/bblogreplay/";

  std::string scenario = "";
  try {
    scenario = config->get_string((prefix + "scenario").c_str());
  } catch (Exception &e) {
    e.append("No scenario defined, configure %sscenario", prefix.c_str());
    throw;
  }

  std::string scenario_prefix = prefix + scenario + "/";
  std::string logs_prefix     = scenario_prefix + "logs/";

  std::string logdir = LOGDIR;
  try {
    logdir = config->get_string((scenario_prefix + "logdir").c_str());
  } catch (Exception &e) { /* ignored, use default set above */ }
  struct stat s;
  int err = stat(logdir.c_str(), &s);
  if (err != 0) {
    char buf[1024];
    Exception se ("Cannot access logdir %s (%s)",
		  logdir.c_str(), strerror_r(errno, buf, 1024));
  } else if ( ! S_ISDIR(s.st_mode) ) {
    throw Exception("Logdir path %s is not a directory", logdir.c_str());
  }

  bool scenario_loop_replay  = false;
  bool scenario_non_blocking = false;
  float scenario_grace_period = 0.001;
  try {
    scenario_loop_replay = config->get_bool((prefix + "loop").c_str());
  } catch (Exception &e) {} // ignored, assume enabled
  try {
    scenario_loop_replay = config->get_bool((scenario_prefix + "loop").c_str());
  } catch (Exception &e) {} // ignored, assume enabled
  try {
    scenario_non_blocking = config->get_bool((prefix + "non_blocking").c_str());
  } catch (Exception &e) {} // ignored, assume enabled
  try {
    scenario_non_blocking = config->get_bool((scenario_prefix + "non_blocking").c_str());
  } catch (Exception &e) {} // ignored, assume enabled
  try {
    scenario_grace_period = config->get_float((prefix + "grace_period").c_str());
  } catch (Exception &e) {} // ignored, assume enabled
  try {
    scenario_grace_period = config->get_float((scenario_prefix + "grace_period").c_str());
  } catch (Exception &e) {} // ignored, assume enabled

#if __cplusplus >= 201103L
  std::unique_ptr<Configuration::ValueIterator> i(config->search(logs_prefix.c_str()));
#else
  std::auto_ptr<Configuration::ValueIterator> i(config->search(logs_prefix.c_str()));
#endif
  while (i->next()) {
    std::string log_name = std::string(i->path()).substr(logs_prefix.length());
    log_name = log_name.substr(0, log_name.find("/"));

    if ( logs.find(log_name) == logs.end() ) {
      std::string log_prefix = logs_prefix + log_name + "/";

      printf("Log name: %s  log_prefix: %s\n", log_name.c_str(), log_prefix.c_str());

      std::string log_file = "";
      bool loop_replay     = scenario_loop_replay;
      bool non_blocking    = scenario_non_blocking;
      float grace_period   = scenario_grace_period;
      std::string hook_str = "";

      try {
	log_file = config->get_string((log_prefix + "file").c_str());
      } catch (Exception &e) {
	throw;
      }

      try {
	loop_replay = config->get_bool((log_prefix + "loop").c_str());
      } catch (Exception &e) {} // ignored, assume enabled
      try {
	non_blocking = config->get_bool((log_prefix + "non_blocking").c_str());
      } catch (Exception &e) {} // ignored, assume enabled
      try {
	hook_str = config->get_string((log_prefix + "hook").c_str());
      } catch (Exception &e) {} // ignored, assume enabled
      try {
	grace_period = config->get_float((log_prefix + "grace_period").c_str());
      } catch (Exception &e) {} // ignored, assume enabled


      if (hook_str != "") {
	BlockedTimingAspect::WakeupHook hook;
	hook = BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP;

	if (hook_str == "pre_loop") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP;
	} else if (hook_str == "sensor_acquire") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE;
	} else if (hook_str == "sensor_prepare") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE;
	} else if (hook_str == "sensor_process") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS;
	} else if (hook_str == "worldstate") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE;
	} else if (hook_str == "think") {
	hook = BlockedTimingAspect::WAKEUP_HOOK_THINK;
	} else if (hook_str == "skill") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_SKILL;
	} else if (hook_str == "act") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_ACT;
	} else if (hook_str == "act_exec") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC;
	} else if (hook_str == "post_loop") {
	  hook = BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP;
	} else {
	  throw Exception("Invalid hook '%s' for %s",
			  hook_str.c_str(), i->get_string().c_str());
	}

	BBLogReplayBlockedTimingThread *lrbt_thread;
	lrbt_thread = new BBLogReplayBlockedTimingThread(hook,
							 i->get_string().c_str(),
							 logdir.c_str(),
							 scenario.c_str(),
							 grace_period,
							 loop_replay,
							 non_blocking);
	thread_list.push_back(lrbt_thread);
      } else {
	BBLogReplayThread *lr_thread = new BBLogReplayThread(i->get_string().c_str(),
							     logdir.c_str(),
							     scenario.c_str(),
							     grace_period,
							     loop_replay);
	thread_list.push_back(lr_thread);
      }

      logs.insert(log_name);
    }
  }

  if ( thread_list.empty() ) {
    throw Exception("No interfaces configured for log replay, aborting");
  }
}

PLUGIN_DESCRIPTION("Replay BlackBoard log files")
EXPORT_PLUGIN(BlackBoardLogReplayPlugin)
