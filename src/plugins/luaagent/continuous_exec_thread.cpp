
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
#include <logging/component.h>

#include <lua/context.h>
#include <lua/interface_importer.h>

#include <interfaces/SkillerInterface.h>
#include <interfaces/SkillerDebugInterface.h>

#include <string>
#include <cstring>

using namespace std;
using namespace fawkes;


LuaAgentContinuousExecutionThread *g_agent_thread = NULL;

static int l_read_interfaces(lua_State *L)
{
  g_agent_thread->read_interfaces();
  return 0;
}

static int l_write_interfaces(lua_State *L)
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
  __lua = NULL;
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
    if ( __skiller_if ) {
      __skiller_if->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
      blackboard->close(__skiller_if);
    }
    delete __lua_ifi;
    delete __lua_thread;
    delete __ifi_mutex;

  } catch (...) {
    // we really screwed up, can't do anything about it, ignore error, logger is
    // initialized since this method is only called from init() which is only called if
    // all aspects had been initialized successfully
    logger->log_error(name(), "Really screwed up while finalizing, aborting cleanup. "
		              "Fawkes is no longer in a clean state. Restart!");
  }
}


void
LuaAgentContinuousExecutionThread::init()
{
  try {
    __cfg_agent       = config->get_string("/luaagent/agent");
    __cfg_watch_files = config->get_bool("/luaagent/watch_files");
  } catch (Exception &e) {
    e.append("Insufficient configuration for LuaAgent");
    throw;
  }

  logger->log_debug("LuaAgentContinuousExecutionThread", "Agent: %s", __cfg_agent.c_str());

  __clog = new ComponentLogger(logger, "LuaAgentLua");

  __lua = NULL;
  __lua_ifi = NULL;
  __lua_thread = NULL;
  __skiller_if = NULL;
  __ifi_mutex = NULL;

  std::string reading_prefix = "/luaagent/interfaces/" + __cfg_agent + "/reading/";
  std::string writing_prefix = "/luaagent/interfaces/" + __cfg_agent + "/writing/";

  __skiller_if = blackboard->open_for_reading<SkillerInterface>("Skiller");

  __skiller_if->read();
  if (__skiller_if->exclusive_controller() != 0) {
    throw Exception("Skiller already has an exclusive controller");
  }

  __skiller_if->msgq_enqueue(new SkillerInterface::AcquireControlMessage());

  try {
    __lua  = new LuaContext();
    if (__cfg_watch_files) {
      __lua->setup_fam(/* auto restart */ false, /* conc thread */ true);
      __lua->get_fam()->add_listener(this);
    }

    __lua_ifi = new LuaInterfaceImporter(__lua, blackboard, config, logger);
    __lua_ifi->open_reading_interfaces(reading_prefix);
    __lua_ifi->open_writing_interfaces(writing_prefix);

    __lua->add_package_dir(LUADIR);
    __lua->add_cpackage_dir(LUALIBDIR);

    __lua->add_package("fawkesutils");
    __lua->add_package("fawkesconfig");
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
    __lua->set_cfunction("read_interfaces", l_read_interfaces);
    __lua->set_cfunction("write_interfaces", l_write_interfaces);

    __lua_ifi->add_interface("skiller", __skiller_if);

    __lua_ifi->read_to_buffer();
    __lua_ifi->push_interfaces();

    __lua->set_start_script(LUADIR"/luaagent/fawkes/start.lua");

    __lua_thread = new LuaThread(__lua);
    thread_collector->add(__lua_thread);

    __ifi_mutex = new Mutex();
  } catch (Exception &e) {
    init_failure_cleanup();
    throw;
  }
}


void
LuaAgentContinuousExecutionThread::finalize()
{
  if (__skiller_if->has_writer() ) {
    __skiller_if->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
  }

  blackboard->close(__skiller_if);

  if (__lua_thread) {
    thread_collector->remove(__lua_thread);
    delete __lua_thread;
  }

  delete __lua_ifi;
  delete __ifi_mutex;
  delete __lua;
  delete __clog;
}


void
LuaAgentContinuousExecutionThread::loop()
{
  __ifi_mutex->lock();

  __lua_ifi->read_to_buffer();
  __skiller_if->read();

  if (__lua_thread && __lua_thread->failed()) {
    logger->log_error(name(), "LuaThread failed, agent died, removing thread");
    thread_collector->remove(__lua_thread);
    delete __lua_thread;
    __lua_thread = NULL;
  }
  __ifi_mutex->unlock();
}


/** Update all reading interfaces.
 * This is meant to be called from inside Lua so that the agent can
 * update the set of interfaces at suitable points in time.
 */
void
LuaAgentContinuousExecutionThread::read_interfaces()
{
  __ifi_mutex->lock();
  logger->log_debug(name(), "Reading interfaces");
  __lua_ifi->read_from_buffer();
  __ifi_mutex->unlock();
}


/** Update all reading interfaces.
 * This is meant to be called from inside Lua so that the agent can
 * update the set of interfaces at suitable points in time.
 */
void
LuaAgentContinuousExecutionThread::write_interfaces()
{
  __ifi_mutex->lock();
  logger->log_debug(name(), "Writing interfaces");
  __lua_ifi->write();
  __ifi_mutex->unlock();
}

void
LuaAgentContinuousExecutionThread::fam_event(const char *filename,
					     unsigned int mask)
{
  if (__lua_thread) {
    __lua_thread->cancel();
    __lua_thread->join();
  }

  __ifi_mutex->lock();
  logger->log_warn(name(), "Restarting Lua context");
  __lua->restart();
  __lua_thread->start();
  __ifi_mutex->unlock();
}


/** Constructor.
 * @param lua Lua context to use
 */
LuaAgentContinuousExecutionThread::LuaThread::LuaThread(fawkes::LuaContext *lua)
  : Thread("LuaAgentContinuousExecutionThread::LuaThread",
	   Thread::OPMODE_CONTINUOUS)
{
  set_prepfin_conc_loop(true);
  __lua = lua;
  __failed = false;
}


/** Loop method continuously calling agentenv.execute() in Lua. */
void
LuaAgentContinuousExecutionThread::LuaThread::loop()
{
  while (!__failed) {
    try {
      // Stack:
      __lua->do_string("agentenv.execute()");
    } catch (Exception &e) {
      __failed = true;
      logger->log_error(name(), "execute() failed, exception follows");
      logger->log_error(name(), e);
    }
  }
}
