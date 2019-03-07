
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
#include <interfaces/SkillerDebugInterface.h>
#include <interfaces/SkillerInterface.h>
#include <logging/component.h>
#include <lua/context.h>
#include <lua/interface_importer.h>

#include <cstring>
#include <string>

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
	lua_ = NULL;
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
		if (skiller_if_) {
			skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
			blackboard->close(skiller_if_);
		}
		if (agdbg_if_)
			blackboard->close(agdbg_if_);

		delete lua_ifi_;

	} catch (...) {
		// we really screwed up, can't do anything about it, ignore error, logger is
		// initialized since this method is only called from init() which is only called if
		// all aspects had been initialized successfully
		logger->log_error(name(),
		                  "Really screwed up while finalizing, aborting cleanup. "
		                  "Fawkes is no longer in a clean state. Restart!");
	}
}

void
LuaAgentPeriodicExecutionThread::init()
{
	try {
		cfg_agent_       = config->get_string("/luaagent/agent");
		cfg_watch_files_ = config->get_bool("/luaagent/watch_files");
	} catch (Exception &e) {
		e.append("Insufficient configuration for LuaAgent");
		throw;
	}

	logger->log_debug("LuaAgentPeriodicExecutionThread", "Agent: %s", cfg_agent_.c_str());

	clog_ = new ComponentLogger(logger, "LuaAgentLua");

	lua_        = NULL;
	lua_ifi_    = NULL;
	skiller_if_ = NULL;
	agdbg_if_   = NULL;

	std::string reading_prefix = "/luaagent/interfaces/" + cfg_agent_ + "/reading/";
	std::string writing_prefix = "/luaagent/interfaces/" + cfg_agent_ + "/writing/";

	skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

	skiller_if_->read();
	if (skiller_if_->exclusive_controller() != 0) {
		throw Exception("Skiller already has an exclusive controller");
	}

	skiller_if_->msgq_enqueue(new SkillerInterface::AcquireControlMessage());
	agdbg_if_ = blackboard->open_for_writing<SkillerDebugInterface>("LuaAgent");

	try {
		lua_ = new LuaContext();
		if (cfg_watch_files_) {
			lua_->setup_fam(/* auto restart */ true, /* conc thread */ false);
		}

		lua_ifi_ = new LuaInterfaceImporter(lua_, blackboard, config, logger);
		lua_ifi_->open_reading_interfaces(reading_prefix);
		lua_ifi_->open_writing_interfaces(writing_prefix);

		lua_->add_package_dir(LUADIR);
		lua_->add_cpackage_dir(LUALIBDIR);

		lua_->add_package("fawkesutils");
		lua_->add_package("fawkesconfig");
		lua_->add_package("fawkeslogging");
		lua_->add_package("fawkesinterface");
#ifdef HAVE_TF
		lua_->add_package("fawkestf");
#endif

		lua_->set_string("AGENT", cfg_agent_.c_str());
		lua_->set_usertype("config", config, "Configuration", "fawkes");
		lua_->set_usertype("logger", clog_, "ComponentLogger", "fawkes");
		lua_->set_usertype("clock", clock, "Clock", "fawkes");
#ifdef HAVE_TF
		lua_->set_usertype("tf", tf_listener, "Transformer", "fawkes::tf");
#endif

		lua_ifi_->add_interface("skiller", skiller_if_);
		lua_ifi_->add_interface("agdbg", agdbg_if_);

		lua_ifi_->push_interfaces();

		lua_->set_start_script(LUADIR "/luaagent/fawkes/start.lua");
	} catch (Exception &e) {
		init_failure_cleanup();
		throw;
	}

	agdbg_if_->set_graph("");
	agdbg_if_->set_graph_fsm(cfg_agent_.c_str());
}

void
LuaAgentPeriodicExecutionThread::finalize()
{
	if (skiller_if_->has_writer()) {
		skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
	}

	blackboard->close(skiller_if_);
	blackboard->close(agdbg_if_);

	delete lua_ifi_;
	delete lua_;
	delete clog_;
}

void
LuaAgentPeriodicExecutionThread::process_agdbg_messages()
{
	while (!agdbg_if_->msgq_empty()) {
		if (agdbg_if_->msgq_first_is<SkillerDebugInterface::SetGraphDirectionMessage>()) {
			SkillerDebugInterface::SetGraphDirectionMessage *m =
			  agdbg_if_->msgq_first<SkillerDebugInterface::SetGraphDirectionMessage>();
			try {
				std::string graphdir = "TB";
				switch (m->graph_dir()) {
				case SkillerDebugInterface::GD_BOTTOM_TOP: graphdir = "BT"; break;
				case SkillerDebugInterface::GD_LEFT_RIGHT: graphdir = "LR"; break;
				case SkillerDebugInterface::GD_RIGHT_LEFT: graphdir = "RL"; break;
				default: break;
				}
				lua_->do_string("agentenv.set_graphdir(\"%s\")", graphdir.c_str());
			} catch (Exception &e) {
				logger->log_warn("LuaAgentPeriodicExecutionThread",
				                 "Failed to set graph direction, exception follows");
				logger->log_warn("LuaAgentPeriodicExecutionThread", e);
			}
		} else if (agdbg_if_->msgq_first_is<SkillerDebugInterface::SetGraphColoredMessage>()) {
			SkillerDebugInterface::SetGraphColoredMessage *m =
			  agdbg_if_->msgq_first<SkillerDebugInterface::SetGraphColoredMessage>();
			try {
				lua_->do_string("agentenv.set_graph_colored(%s)", m->is_graph_colored() ? "true" : "false");
			} catch (Exception &e) {
				logger->log_warn("LuaAgentPeriodicExecutionThread",
				                 "Failed to set graph direction, exception follows");
				logger->log_warn("LuaAgentPeriodicExecutionThread", e);
			}
		}

		agdbg_if_->msgq_pop();
	}
}

void
LuaAgentPeriodicExecutionThread::loop()
{
#ifdef HAVE_INOTIFY
	lua_->process_fam_events();
#endif

	process_agdbg_messages();

	lua_ifi_->read();
	skiller_if_->read();

	try {
		// Stack:
		lua_->do_string("agentenv.execute()");
	} catch (Exception &e) {
		logger->log_error("LuaAgentPeriodicExecutionThread",
		                  "Execution of %s.execute() failed, exception follows",
		                  cfg_agent_.c_str());
		logger->log_error("LuaAgentPeriodicExecutionThread", e);
	}

	lua_ifi_->write();
}
