
/***************************************************************************
 *  exec_thread.cpp - Fawkes LuaAgent: Execution Thread
 *
 *  Created: Thu Jan 01 11:12:13 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include "exec_thread.h"

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <utils/logging/component.h>

#include <lua/context.h>
#include <lua/interface_importer.h>

#include <interfaces/SkillerInterface.h>
#include <interfaces/SkillerDebugInterface.h>

#include <string>
#include <cstring>

using namespace std;
using namespace fawkes;

/** @class LuaAgentExecutionThread "exec_thread.h"
 * LuaAgent Execution Thread.
 * This thread runs and controls the Lua interpreter and passes data into the
 * execution engine.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
LuaAgentExecutionThread::LuaAgentExecutionThread()
  : Thread("LuaAgentExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK)
{
  __lua = NULL;
}


/** Destructor. */
LuaAgentExecutionThread::~LuaAgentExecutionThread()
{
}


/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
LuaAgentExecutionThread::init_failure_cleanup()
{
  try {
    if ( __skiller_if ) blackboard->close(__skiller_if);
    if ( __agdbg_if )   blackboard->close(__agdbg_if);

    delete __lua_ifi;

  } catch (...) {
    // we really screwed up, can't do anything about it, ignore error, logger is
    // initialized since this method is only called from init() which is only called if
    // all aspects had been initialized successfully
    logger->log_error(name(), "Really screwed up while finalizing, aborting cleanup. "
		              "Fawkes is no longer in a clean state. Restart!");
  }
}


void
LuaAgentExecutionThread::init()
{
  try {
    __cfg_agent       = config->get_string("/luaagent/agent");
    __cfg_watch_files = config->get_bool("/luaagent/watch_files");
  } catch (Exception &e) {
    e.append("Insufficient configuration for LuaAgent");
    throw;
  }

  logger->log_debug("LuaAgentExecutionThread", "Agent: %s", __cfg_agent.c_str());

  __clog = new ComponentLogger(logger, "LuaAgentLua");

  __lua = NULL;
  __lua_ifi = NULL;
  __skiller_if = NULL;
  __agdbg_if = NULL;

  std::string reading_prefix = "/luaagent/interfaces/" + __cfg_agent + "/reading/";
  std::string writing_prefix = "/luaagent/interfaces/" + __cfg_agent + "/writing/";

  __skiller_if = blackboard->open_for_reading<SkillerInterface>("Skiller");

  __skiller_if->read();
  if (__skiller_if->exclusive_controller() != 0) {
    throw Exception("Skiller already has an exclusive controller");
  }

  __skiller_if->msgq_enqueue(new SkillerInterface::AcquireControlMessage());
  __agdbg_if   = blackboard->open_for_writing<SkillerDebugInterface>("LuaAgent");

  try {
    __lua  = new LuaContext(__cfg_watch_files);

    __lua_ifi = new LuaInterfaceImporter(__lua, blackboard, config, logger);
    __lua_ifi->open_reading_interfaces(reading_prefix);
    __lua_ifi->open_writing_interfaces(writing_prefix);

    __lua->add_package_dir(LUADIR);
    __lua->add_cpackage_dir(LUALIBDIR);

    __lua->add_package("fawkesutils");
    __lua->add_package("fawkesconfig");
    __lua->add_package("fawkesinterface");

    __lua->set_string("AGENT", __cfg_agent.c_str());
    __lua->set_usertype("config", config, "Configuration", "fawkes");
    __lua->set_usertype("logger", __clog, "ComponentLogger", "fawkes");
    __lua->set_usertype("clock", clock, "Clock", "fawkes");

    __lua_ifi->add_interface("skiller", __skiller_if);
    __lua_ifi->add_interface("agdbg", __agdbg_if);

    __lua_ifi->push_interfaces();

    __lua->set_start_script(LUADIR"/luaagent/start.lua");
  } catch (Exception &e) {
    init_failure_cleanup();
    throw;
  }

  __agdbg_if->set_graph("");
  __agdbg_if->set_graph_fsm(__cfg_agent.c_str());

}


void
LuaAgentExecutionThread::finalize()
{
  if (__skiller_if->has_writer() ) {
    __skiller_if->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
  }

  blackboard->close(__skiller_if);
  blackboard->close(__agdbg_if);

  delete __lua_ifi;
  delete __lua;
  delete __clog;
}

void
LuaAgentExecutionThread::process_agdbg_messages()
{
  while ( ! __agdbg_if->msgq_empty() ) {
    if (__agdbg_if->msgq_first_is<SkillerDebugInterface::SetGraphDirectionMessage>() ) {
      SkillerDebugInterface::SetGraphDirectionMessage *m = __agdbg_if->msgq_first<SkillerDebugInterface::SetGraphDirectionMessage>();
      try {
	std::string graphdir = "TB";
	switch (m->graph_dir()) {
	case SkillerDebugInterface::GD_BOTTOM_TOP: graphdir = "BT"; break;
	case SkillerDebugInterface::GD_LEFT_RIGHT: graphdir = "LR"; break;
	case SkillerDebugInterface::GD_RIGHT_LEFT: graphdir = "RL"; break;
	default: break;
	}
	__lua->do_string("agentenv.set_graphdir(\"%s\")", graphdir.c_str());
      } catch (Exception &e) {
	logger->log_warn("LuaAgentExecutionThread", "Failed to set graph direction, exception follows");
	logger->log_warn("LuaAgentExecutionThread", e);
      }
    } else if (__agdbg_if->msgq_first_is<SkillerDebugInterface::SetGraphColoredMessage>() ) {
      SkillerDebugInterface::SetGraphColoredMessage *m = __agdbg_if->msgq_first<SkillerDebugInterface::SetGraphColoredMessage>();
      try {
	__lua->do_string("agentenv.set_graph_colored(%s)", m->is_graph_colored() ? "true" : "false");
      } catch (Exception &e) {
	logger->log_warn("LuaAgentExecutionThread", "Failed to set graph direction, exception follows");
	logger->log_warn("LuaAgentExecutionThread", e);
      }
    }

    __agdbg_if->msgq_pop();
  }
}


void
LuaAgentExecutionThread::loop()
{
#ifdef HAVE_INOTIFY
  __lua->process_fam_events();
#endif

  process_agdbg_messages();

  __lua_ifi->read();
  __skiller_if->read();

  try {
    // Stack:
    __lua->do_string("agentenv.execute()");
  } catch (Exception &e) {
    logger->log_error("LuaAgentExecutionThread", "Execution of %s.execute() failed, exception follows",
		      __cfg_agent.c_str());
    logger->log_error("LuaAgentExecutionThread", e);
  }

  __lua_ifi->write();
}
