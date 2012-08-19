
/***************************************************************************
 *  periodic_exec_thread.cpp - Fawkes LuaAgent: Periodic Execution Thread
 *
 *  Created: Thu Jan 01 11:12:13 2009
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "periodic_exec_thread.h"

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <logging/component.h>

#include <lua/context.h>
#include <lua/interface_importer.h>

#include <interfaces/SkillerInterface.h>
#include <interfaces/SkillerDebugInterface.h>

#include <string>
#include <cstring>

using namespace std;
using namespace fawkes;

/** @class LuaAgentPeriodicExecutionThread "periodic_exec_thread.h"
 * LuaAgent Periodic Execution Thread.
 * This thread runs and controls the Lua interpreter and passes data into the
 * execution engine. It hooks into the THINK main loop hook and expects the
 * agent's execution function to return quickly. If you have a separate agent
 * main loop use the concurrent execution thread.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
LuaAgentPeriodicExecutionThread::LuaAgentPeriodicExecutionThread()
  : Thread("LuaAgentPeriodicExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK)
{
  __lua = NULL;
}


/** Destructor. */
LuaAgentPeriodicExecutionThread::~LuaAgentPeriodicExecutionThread()
{
}


/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
LuaAgentPeriodicExecutionThread::init_failure_cleanup()
{
  try {
    if ( __skiller_if ) {
      __skiller_if->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
      blackboard->close(__skiller_if);
    }
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
LuaAgentPeriodicExecutionThread::init()
{
  try {
    __cfg_agent       = config->get_string("/luaagent/agent");
    __cfg_watch_files = config->get_bool("/luaagent/watch_files");
  } catch (Exception &e) {
    e.append("Insufficient configuration for LuaAgent");
    throw;
  }

  logger->log_debug("LuaAgentPeriodicExecutionThread", "Agent: %s", __cfg_agent.c_str());

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
    __lua  = new LuaContext();
    if (__cfg_watch_files) {
      __lua->setup_fam(/* auto restart */ true, /* conc thread */ false);
    }

    __lua_ifi = new LuaInterfaceImporter(__lua, blackboard, config, logger);
    __lua_ifi->open_reading_interfaces(reading_prefix);
    __lua_ifi->open_writing_interfaces(writing_prefix);

    __lua->add_package_dir(LUADIR);
    __lua->add_cpackage_dir(LUALIBDIR);

    __lua->add_package("fawkesutils");
    __lua->add_package("fawkesconfig");
    __lua->add_package("fawkeslogging");
    __lua->add_package("fawkesinterface");
    __lua->add_package("fawkesgeometry");
#ifdef HAVE_TF
    __lua->add_package("fawkestf");
#endif

    __lua->set_string("AGENT", __cfg_agent.c_str());
    __lua->set_usertype("config", config, "Configuration", "fawkes");
    __lua->set_usertype("logger", __clog, "ComponentLogger", "fawkes");
    __lua->set_usertype("clock", clock, "Clock", "fawkes");
#ifdef HAVE_TF
    __lua->set_usertype("tf", tf_listener, "Transformer", "fawkes::tf");
#endif

    __lua_ifi->add_interface("skiller", __skiller_if);
    __lua_ifi->add_interface("agdbg", __agdbg_if);

    __lua_ifi->push_interfaces();

    __lua->set_start_script(LUADIR"/luaagent/fawkes/start.lua");
  } catch (Exception &e) {
    init_failure_cleanup();
    throw;
  }

  __agdbg_if->set_graph("");
  __agdbg_if->set_graph_fsm(__cfg_agent.c_str());

}


void
LuaAgentPeriodicExecutionThread::finalize()
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
LuaAgentPeriodicExecutionThread::process_agdbg_messages()
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
	logger->log_warn("LuaAgentPeriodicExecutionThread", "Failed to set graph direction, exception follows");
	logger->log_warn("LuaAgentPeriodicExecutionThread", e);
      }
    } else if (__agdbg_if->msgq_first_is<SkillerDebugInterface::SetGraphColoredMessage>() ) {
      SkillerDebugInterface::SetGraphColoredMessage *m = __agdbg_if->msgq_first<SkillerDebugInterface::SetGraphColoredMessage>();
      try {
	__lua->do_string("agentenv.set_graph_colored(%s)", m->is_graph_colored() ? "true" : "false");
      } catch (Exception &e) {
	logger->log_warn("LuaAgentPeriodicExecutionThread", "Failed to set graph direction, exception follows");
	logger->log_warn("LuaAgentPeriodicExecutionThread", e);
      }
    }

    __agdbg_if->msgq_pop();
  }
}


void
LuaAgentPeriodicExecutionThread::loop()
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
    logger->log_error("LuaAgentPeriodicExecutionThread", "Execution of %s.execute() failed, exception follows",
		      __cfg_agent.c_str());
    logger->log_error("LuaAgentPeriodicExecutionThread", e);
  }

  __lua_ifi->write();
}
