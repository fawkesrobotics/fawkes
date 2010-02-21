
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
  std::set<std::string> ifaces;

  std::string prefix = "/fawkes/bblogreplay/";

  std::string scenario = "";
  try {
    scenario = config->get_string((prefix + "scenario").c_str());
  } catch (Exception &e) {
    e.append("No scenario defined, configure %sscenario", prefix.c_str());
    throw;
  }

  std::string scenario_prefix = prefix + scenario + "/";
  std::string log_prefix      = scenario_prefix + "log/";
  std::string loop_prefix     = scenario_prefix + "loop/";
  std::string hook_prefix     = scenario_prefix + "hook/";

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

  bool loop_replay = false;
  try {
    loop_replay = config->get_bool((scenario_prefix + std::string("loop")).c_str());
  } catch (Exception &e) { /* ignored, use default set above */ }
    
  Configuration::ValueIterator *i = config->search(log_prefix.c_str());
  while (i->next()) {
    bool loop_replay_i = loop_replay;
    std::string hook_str = "";
    char * log_reference = (char*) strrchr(i->path(),'/');
    if(log_reference != NULL){
      log_reference++;
	
      try {
	loop_replay_i = config->get_bool((loop_prefix + std::string(log_reference)).c_str());
      } catch (Exception &e) { /* ignored, use default set above */ }
      try {
	hook_str = config->get_string((hook_prefix + std::string(log_reference)).c_str());
	printf("%s = %s\n", (hook_prefix + std::string(log_reference)).c_str(), hook_str.c_str());
      } catch (Exception &e) { /* ignored, use default set above */ }
    }

    if (hook_str != "") {
      BlockedTimingAspect::WakeupHook hook;
      hook = BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP;

      if (hook_str == "pre_loop") {
	hook = BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP;
      } else if (hook_str == "sensor") {
	hook = BlockedTimingAspect::WAKEUP_HOOK_SENSOR;
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
							 loop_replay_i);
      thread_list.push_back(lrbt_thread);
    } else {
      BBLogReplayThread *lr_thread = new BBLogReplayThread(i->get_string().c_str(),
							   logdir.c_str(),
							   scenario.c_str(),
							   loop_replay_i);
      thread_list.push_back(lr_thread);
    }

  }
  delete i;

  if ( thread_list.empty() ) {
    throw Exception("No interfaces configured for logging, aborting");
  }
}

PLUGIN_DESCRIPTION("Replay BlackBoard log files")
EXPORT_PLUGIN(BlackBoardLogReplayPlugin)

