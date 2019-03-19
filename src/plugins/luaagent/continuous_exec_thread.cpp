
/***************************************************************************
 *  continuous_exec_thread.cpp - Fawkes LuaAgent: Continuous Execution Thread
 *
 *  Created: Thu May 26 11:50:15 2011
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

#include "continuous_exec_thread.h"

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

LuaAgentContinuousExecutionThread *g_agent_thread = NULL;

static int
l_read_interfaces(lua_State *L)
{
	g_agent_thread->read_interfaces();
	return 0;
}

static int
l_write_interfaces(lua_State *L)
{
	g_agent_thread->write_interfaces();
	return 0;
}

/** @class LuaAgentContinuousExecutionThread "periodic_exec_thread.h"
 * LuaAgent Periodic Execution Thread.
 * This thread runs and controls the Lua interpreter and passes data into the
 * execution engine. It hooks into the THINK main loop hook and expects the
 * agent's execution function to return quickly. If you have a separate agent
 * main loop use the concurrent execution thread.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
LuaAgentContinuousExecutionThread::LuaAgentContinuousExecutionThread()
: Thread("LuaAgentContinuousExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK)
{
	lua_ = NULL;
	if (g_agent_thread != NULL) {
		throw Exception("A global thread has already been set");
	}
	g_agent_thread = this;
}

/** Destructor. */
LuaAgentContinuousExecutionThread::~LuaAgentContinuousExecutionThread()
{
	g_agent_thread = NULL;
}

/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
LuaAgentContinuousExecutionThread::init_failure_cleanup()
{
	try {
		if (skiller_if_) {
			skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
			blackboard->close(skiller_if_);
		}
		delete lua_ifi_;
		delete lua_thread_;
		delete ifi_mutex_;

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
LuaAgentContinuousExecutionThread::init()
{
	try {
		cfg_agent_       = config->get_string("/luaagent/agent");
		cfg_watch_files_ = config->get_bool("/luaagent/watch_files");
	} catch (Exception &e) {
		e.append("Insufficient configuration for LuaAgent");
		throw;
	}

	logger->log_debug("LuaAgentContinuousExecutionThread", "Agent: %s", cfg_agent_.c_str());

	clog_ = new ComponentLogger(logger, "LuaAgentLua");

	lua_        = NULL;
	lua_ifi_    = NULL;
	lua_thread_ = NULL;
	skiller_if_ = NULL;
	ifi_mutex_  = NULL;

	std::string reading_prefix = "/luaagent/interfaces/" + cfg_agent_ + "/reading/";
	std::string writing_prefix = "/luaagent/interfaces/" + cfg_agent_ + "/writing/";

	skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

	skiller_if_->read();
	if (skiller_if_->exclusive_controller() != 0) {
		throw Exception("Skiller already has an exclusive controller");
	}

	skiller_if_->msgq_enqueue(new SkillerInterface::AcquireControlMessage());

	try {
		lua_ = new LuaContext();
		if (cfg_watch_files_) {
			lua_->setup_fam(/* auto restart */ false, /* conc thread */ true);
			lua_->get_fam()->add_listener(this);
		}

		lua_ifi_ = new LuaInterfaceImporter(lua_, blackboard, config, logger);
		lua_ifi_->open_reading_interfaces(reading_prefix);
		lua_ifi_->open_writing_interfaces(writing_prefix);

		lua_->add_package_dir(LUADIR);
		lua_->add_cpackage_dir(LUALIBDIR);

		lua_->add_package("fawkesutils");
		lua_->add_package("fawkesconfig");
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
		lua_->set_cfunction("read_interfaces", l_read_interfaces);
		lua_->set_cfunction("write_interfaces", l_write_interfaces);

		lua_ifi_->add_interface("skiller", skiller_if_);

		lua_ifi_->read_to_buffer();
		lua_ifi_->push_interfaces();

		lua_->set_start_script(LUADIR "/luaagent/fawkes/start.lua");

		lua_thread_ = new LuaThread(lua_);
		thread_collector->add(lua_thread_);

		ifi_mutex_ = new Mutex();
	} catch (Exception &e) {
		init_failure_cleanup();
		throw;
	}
}

void
LuaAgentContinuousExecutionThread::finalize()
{
	if (skiller_if_->has_writer()) {
		skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
	}

	blackboard->close(skiller_if_);

	if (lua_thread_) {
		thread_collector->remove(lua_thread_);
		delete lua_thread_;
	}

	delete lua_ifi_;
	delete ifi_mutex_;
	delete lua_;
	delete clog_;
}

void
LuaAgentContinuousExecutionThread::loop()
{
	ifi_mutex_->lock();

	lua_ifi_->read_to_buffer();
	skiller_if_->read();

	if (lua_thread_ && lua_thread_->failed()) {
		logger->log_error(name(), "LuaThread failed, agent died, removing thread");
		thread_collector->remove(lua_thread_);
		delete lua_thread_;
		lua_thread_ = NULL;
	}
	ifi_mutex_->unlock();
}

/** Update all reading interfaces.
 * This is meant to be called from inside Lua so that the agent can
 * update the set of interfaces at suitable points in time.
 */
void
LuaAgentContinuousExecutionThread::read_interfaces()
{
	ifi_mutex_->lock();
	logger->log_debug(name(), "Reading interfaces");
	lua_ifi_->read_from_buffer();
	ifi_mutex_->unlock();
}

/** Update all reading interfaces.
 * This is meant to be called from inside Lua so that the agent can
 * update the set of interfaces at suitable points in time.
 */
void
LuaAgentContinuousExecutionThread::write_interfaces()
{
	ifi_mutex_->lock();
	logger->log_debug(name(), "Writing interfaces");
	lua_ifi_->write();
	ifi_mutex_->unlock();
}

void
LuaAgentContinuousExecutionThread::fam_event(const char *filename, unsigned int mask)
{
	if (lua_thread_) {
		lua_thread_->cancel();
		lua_thread_->join();
	}

	ifi_mutex_->lock();
	logger->log_warn(name(), "Restarting Lua context");
	lua_->restart();
	lua_thread_->start();
	ifi_mutex_->unlock();
}

/** Constructor.
 * @param lua Lua context to use
 */
LuaAgentContinuousExecutionThread::LuaThread::LuaThread(fawkes::LuaContext *lua)
: Thread("LuaAgentContinuousExecutionThread::LuaThread", Thread::OPMODE_CONTINUOUS)
{
	set_prepfin_conc_loop(true);
	lua_    = lua;
	failed_ = false;
}

/** Loop method continuously calling agentenv.execute() in Lua. */
void
LuaAgentContinuousExecutionThread::LuaThread::loop()
{
	while (!failed_) {
		try {
			// Stack:
			lua_->do_string("agentenv.execute()");
		} catch (Exception &e) {
			failed_ = true;
			logger->log_error(name(), "execute() failed, exception follows");
			logger->log_error(name(), e);
		}
	}
}
