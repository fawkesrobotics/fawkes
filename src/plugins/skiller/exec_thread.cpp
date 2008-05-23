
/***************************************************************************
 *  exec_thread.cpp - Fawkes Skiller: Execution Thread
 *
 *  Created: Mon Feb 18 10:30:17 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <plugins/skiller/exec_thread.h>
#include <plugins/skiller/liaison_thread.h>

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <core/threading/barrier.h>
#include <utils/logging/component.h>

#include <interfaces/object.h>
#include <interfaces/skiller.h>
#include <interfaces/navigator.h>
#include <interfaces/gamestate.h>

#include <lua.hpp>
#include <tolua++.h>

#include <string>
#include <cstring>

#define INIT_FILE  SKILLDIR"/general/init.lua"
#define START_FILE SKILLDIR"/general/start.lua"

using namespace std;
using namespace fawkes;

/** @class SkillerExecutionThread <plugins/skiller/exec_thread.h>
 * Skiller Execution Thread.
 * This thread runs and controls the Lua interpreter and passes data into the
 * execution engine. It can act upon signals from the liaison thread if
 * necessary.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param liaison_exec_barrier Barrier used to synchronize liaison and exec thread
 * @param slt Skiller liaison thread
 */
SkillerExecutionThread::SkillerExecutionThread(Barrier *liaison_exec_barrier,
					       SkillerLiaisonThread *slt)
  : Thread("SkillerExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
  __liaison_exec_barrier = liaison_exec_barrier;
  __slt = slt;
  __L = NULL;

  __continuous_run = false;

  __fam = NULL;
}


/** Destructor. */
SkillerExecutionThread::~SkillerExecutionThread()
{
}


void
SkillerExecutionThread::init_lua()
{
  lua_State *tL = luaL_newstate();
  luaL_openlibs(tL);

  lua_pushstring(tL, SKILLDIR);                  lua_setglobal(tL, "SKILLDIR");
  lua_pushstring(tL, LIBDIR);                    lua_setglobal(tL, "LIBDIR");

  lua_pushstring(tL, __cfg_skillspace.c_str());  lua_setglobal(tL, "SKILLSPACE");

  // Load initialization code
  if ( (__err = luaL_loadfile(tL, INIT_FILE)) != 0) {
    __errmsg = lua_tostring(tL, -1);
    lua_pop(tL, 1);
    lua_close(tL);
    switch (__err) {
    case LUA_ERRSYNTAX:
      throw SyntaxErrorException("Lua syntax error: %s", __errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not load Lua init file");

    case LUA_ERRFILE:
      throw CouldNotOpenFileException(INIT_FILE, __errmsg.c_str());
    }
  }

  if ( (__err = lua_pcall(tL, 0, 0, 0)) != 0 ) {
    // There was an error while executing the initialization file
    __errmsg = lua_tostring(tL, -1);
    lua_pop(tL, 1);
    lua_close(tL);
    switch (__err) {
    case LUA_ERRRUN:
      throw Exception("Lua runtime error: %s", __errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not execute Lua init file");

    case LUA_ERRERR:
      throw Exception("Failed to execute error handler during error: %s", __errmsg.c_str());
    }
  }

  // Export some utilities to Lua
  // NOTE: all the (tLua) types that you use here must have been declared before, probably
  // by having an appropriate require clause for a wrapper in init.lua!
  tolua_pushusertype(tL, config, "fawkes::Configuration");
  lua_setglobal(tL, "config");

  tolua_pushusertype(tL, __clog, "fawkes::ComponentLogger");
  lua_setglobal(tL, "logger");

  tolua_pushusertype(tL, clock, "fawkes::Clock");
  lua_setglobal(tL, "clock");

  // Make sure Lua is not currently being executed
  __lua_mutex->lock();
  if ( __L != NULL ) {
    lua_close(__L);
  }
  __L = tL;

  __continuous_run = false;

  __lua_mutex->unlock();
}


void
SkillerExecutionThread::start_lua()
{
  unsigned int tmp_size = 64;
  char tmp[tmp_size];
  // Get interfaces from liaison thread

  strcpy(tmp, "fawkes::");
  tolua_pushusertype(__L, __slt->wm_ball, strncat(tmp, __slt->wm_ball->type(), tmp_size));
  lua_setglobal(__L, "wm_ball");

  strcpy(tmp, "fawkes::");
  tolua_pushusertype(__L, __slt->wm_pose, strncat(tmp, __slt->wm_pose->type(), tmp_size));
  lua_setglobal(__L, "wm_pose");

  strcpy(tmp, "fawkes::");
  tolua_pushusertype(__L, __slt->navigator, strncat(tmp, __slt->navigator->type(), tmp_size));
  lua_setglobal(__L, "navigator");

  strcpy(tmp, "fawkes::");
  tolua_pushusertype(__L, __slt->gamestate, strncat(tmp, __slt->gamestate->type(), tmp_size));
  lua_setglobal(__L, "gamestate");

  // Load start code
  if ( (__err = luaL_loadfile(__L, START_FILE)) != 0) {
    __errmsg = lua_tostring(__L, -1);
    lua_pop(__L, 1);
    switch (__err) {
    case LUA_ERRSYNTAX:
      logger->log_debug("SkillerExecutionThread", "Lua syntax error: %s", __errmsg.c_str());
      break;

    case LUA_ERRMEM:
      logger->log_debug("SkillerExecutionThread", "Lua: Out of memory, cannot load start file");
      break;

    case LUA_ERRFILE:
      logger->log_debug("SkillerExecutionThread", "Lua: could not open start file (%s)", __errmsg.c_str());
      break;

    default:
      logger->log_debug("SkillerExecutionThread", "Lua: unknown error occured (%s)", __errmsg.c_str());
      break;
    }
  } else {
    if ( (__err = lua_pcall(__L, 0, 0, 0)) != 0 ) {
      // There was an error while executing the initialization file
      __errmsg = lua_tostring(__L, -1);
      lua_pop(__L, 1);
      switch (__err) {
      case LUA_ERRRUN:
	logger->log_debug("SkillerExecutionThread", "Lua runtime error: %s", __errmsg.c_str());
	break;
	
      case LUA_ERRMEM:
	logger->log_debug("SkillerExecutionThread", "Lua: Out of memory, cannot execute start file");
	break;

      case LUA_ERRERR:
	logger->log_debug("SkillerExecutionThread", "Lua: Execution failed, error handled failed as well (%s)", __errmsg.c_str());
	break;

      default:
	logger->log_debug("SkillerExecutionThread", "Lua: unknown error occured (%s)", __errmsg.c_str());
	break;
      }
    }
  }
}


void
SkillerExecutionThread::restart_lua()
{
  try {
    init_lua();
    start_lua();
  } catch (Exception &e) {
    logger->log_error("SkillerExecutionThread", "Failed to restart Lua, exception follows");
    logger->log_error("SkillerExecutionThread", e);
  }
}


/** Determines the skill status and writes it to the BB.
 * This method assumes that it is called from within loop() and lua_mutex is locked.
 * @param curss current skill string
 */
void
SkillerExecutionThread::publish_skill_status(std::string &curss)
{
  __slt->skiller->set_skill_string(curss.c_str());
  __slt->skiller->set_continuous(__continuous_run);

  const char *sst = "Unknown";
  LUA_INTEGER running = 0, final = 0, failed = 0;

  if ( luaL_dostring(__L, "return general.skillenv.get_status()") != 0 ) {
    __errmsg = lua_tostring(__L, -1);
    logger->log_error("SkillerExecutionThread", "Failed to get skill status: %s", __errmsg.c_str());
  } else {
    running = lua_tointeger(__L, -3);
    final   = lua_tointeger(__L, -2);
    failed  = lua_tointeger(__L, -1);

    if ( failed > 0 ) {
      sst = "S_FAILED";
      __slt->skiller->set_status(SkillerInterface::S_FAILED);
    } else if ( (final > 0) && (running == 0) ) {
      sst = "S_FINAL";
      __slt->skiller->set_status(SkillerInterface::S_FINAL);
    } else if ( running > 0 ) {
      sst = "S_RUNNING";
      __slt->skiller->set_status(SkillerInterface::S_RUNNING);
    } else {
      // all zero
      sst = "S_INACTIVE";
      __slt->skiller->set_status(SkillerInterface::S_INACTIVE);
    }
  }

  logger->log_debug("SkillerExecutionThread", "Status is %s "
		    "(running %i, final: %i, failed: %i)",
		    sst, running, final, failed);

  __slt->skiller->write();
}


void
SkillerExecutionThread::init()
{
  try {
    __cfg_skillspace  = config->get_string("/skiller/skillspace");
    __cfg_watch_files = config->get_bool("/skiller/watch_files");
  } catch (Exception &e) {
    e.append("Insufficient configuration for Skiller");
    throw;
  }

  __fam = new FileAlterationMonitor();
  __fam->add_listener(this);
  __fam->add_filter("^[^.].*\\.lua$");
  __fam->watch_dir(SKILLDIR);

  __clog = new ComponentLogger(logger, "SkillerLua");
  __lua_mutex = new Mutex();

  try {
    init_lua();
  } catch (Exception &e) {
    delete __clog;
    delete __lua_mutex;
    delete __fam;
    throw;
  }

  logger->log_debug("SkillerExecutionThread", "Skill space: %s", __cfg_skillspace.c_str());
}


void
SkillerExecutionThread::finalize()
{
  lua_close(__L);
  delete __clog;
  __clog = NULL;
  delete __lua_mutex;
  __lua_mutex = NULL;
  delete __fam;
  __fam = NULL;
}


void
SkillerExecutionThread::once()
{
  start_lua();
}


/** Called when reader has been removed from skiller interface.
 * @param instance_serial instance serial of the interface that triggered the event.
 */
void
SkillerExecutionThread::skiller_reader_removed(unsigned int instance_serial)
{
  if ( instance_serial == __slt->skiller->exclusive_controller() ) {
    logger->log_debug("SkillerExecutionThread", "Controlling interface instance was closed, "
		      "revoking exclusive control");
    __slt->skiller->set_exclusive_controller(0);
    __slt->skiller->write();
  }
}

void
SkillerExecutionThread::loop()
{
#ifdef HAVE_INOTIFY
  __fam->process_events();
#endif

  __liaison_exec_barrier->wait();

  // Current skill string
  std::string curss = "";

  unsigned int excl_ctrl  = __slt->skiller->exclusive_controller();

  bool        continuous_reset = false;

  while ( ! __slt->skiller->msgq_empty() ) {
    if ( __slt->skiller->msgq_first_is<SkillerInterface::AcquireControlMessage>() ) {
      Message *m = __slt->skiller->msgq_first();
      if ( excl_ctrl == 0 ) {
	logger->log_debug("SkillerExecutionThread", "%s is new exclusive controller",
			  m->sender_thread_name());
	__slt->skiller->set_exclusive_controller(m->sender_id());
	excl_ctrl = m->sender_id();
      } else {
	logger->log_warn("SkillerExecutionThread", "%s tried to acquire exclusive control, "
			 "but another controller exists already", m->sender_thread_name());
      }

    } else if ( __slt->skiller->msgq_first_is<SkillerInterface::ReleaseControlMessage>() ) {
      Message *m = __slt->skiller->msgq_first();
      if ( excl_ctrl == m->sender_id() ) {
	logger->log_debug("SkillerExecutionThread", "%s releases exclusive control",
			  m->sender_thread_name());

	if ( __continuous_run ) {
	  __continuous_run = false;
	  continuous_reset = true;
	}
	__slt->skiller->set_exclusive_controller(0);
	excl_ctrl = 0;
      } else {
	logger->log_warn("SkillerExecutionThread", "%s tried to release exclusive control, "
			 "it's not the controller", m->sender_thread_name());
      }
    } else if ( __slt->skiller->msgq_first_is<SkillerInterface::ExecSkillMessage>() ) {
      SkillerInterface::ExecSkillMessage *m = __slt->skiller->msgq_first<SkillerInterface::ExecSkillMessage>();

      if ( m->sender_id() == excl_ctrl ) {
	if ( curss != "" ) {
	  logger->log_warn("SkillerExecutionThread", "More than one skill string enqueued, "
			   "ignoring successive string (%s).", m->skill_string());
	} else {	  
	  logger->log_debug("SkillerExecutionThread", "%s wants me to execute '%s'",
			    m->sender_thread_name(), m->skill_string());

	  if ( __continuous_run ) {
	    __continuous_run = false;
	    continuous_reset = true;
	  }
	  curss = m->skill_string();
	}
      } else {
	logger->log_debug("SkillerExecutionThread", "%s tries to exec while not controller",
			  m->sender_thread_name());
      }

    } else if ( __slt->skiller->msgq_first_is<SkillerInterface::ExecSkillContinuousMessage>() ) {
      SkillerInterface::ExecSkillContinuousMessage *m = __slt->skiller->msgq_first<SkillerInterface::ExecSkillContinuousMessage>();

      if ( m->sender_id() == excl_ctrl ) {
	if ( curss != "" ) {
	  logger->log_warn("SkillerExecutionThread", "More than one skill string enqueued, "
			   "ignoring successive string (%s).", m->skill_string());
	} else {	  
	  logger->log_debug("SkillerExecutionThread", "%s wants me to execute '%s'",
			    m->sender_thread_name(), m->skill_string());

	  curss = m->skill_string();
	  continuous_reset = __continuous_run; // reset if cont exec was in progress
	  __continuous_run = true;
	}
      } else {
	logger->log_debug("SkillerExecutionThread", "%s tries to exec while not controller",
			  m->sender_thread_name());
      }

    } else if ( __slt->skiller->msgq_first_is<SkillerInterface::StopExecMessage>() ) {
      SkillerInterface::StopExecMessage *m = __slt->skiller->msgq_first<SkillerInterface::StopExecMessage>();

      if ( m->sender_id() == excl_ctrl ) {
	logger->log_debug("SkillerExecutionThread", "Stopping continuous execution");
	if ( __continuous_run ) {
	  __continuous_run = false;
	  continuous_reset = true;
	}
      } else {
	logger->log_debug("SkillerExecutionThread", "%s tries to stop exec while not controller",
			  m->sender_thread_name());
      }
    } else {
      logger->log_warn("SkillerExecutionThread", "Unhandled message of type %s in "
		       "skiller interface", __slt->skiller->msgq_first()->type());
    }

    __slt->skiller->msgq_pop();
  }

  if ( __continuous_run && (curss == "") ) {
    curss = __slt->skiller->skill_string();
  }

  __lua_mutex->lock();
  if ( continuous_reset ) {
    logger->log_debug("SkillerExecutionThread", "Continuous reset forced");
    luaL_dostring(__L, "general.skillenv.reset_all()");
  }

  if ( curss != "" ) {
    // We've got something to execute

    // we're in continuous mode, reset status for this new loop
    if ( __continuous_run && ! continuous_reset) {
      // was continuous execution, status has to be cleaned up anyway
      logger->log_debug("SkillerExecutionThread", "Resetting skill status in continuous mode");
      luaL_dostring(__L, "general.skillenv.reset_status()");
    }

    if ( (__err = luaL_loadstring(__L, curss.c_str())) != 0 ) {// sksf (skill string function)
      __errmsg = lua_tostring(__L, -1);
      lua_pop(__L, 1);
      switch (__err) {
      case LUA_ERRSYNTAX:
	logger->log_error("SkillerExecutionThread", "Lua syntax error: %s", __errmsg.c_str());
	break;

      case LUA_ERRMEM:
	logger->log_error("SkillerExecutionThread", "Lua ran out of memory");
	break;

      }
    } else {
      luaL_dostring(__L, "return general.skillenv.gensandbox()");  // sksf sandbox
      lua_setfenv(__L, -2);                                        // sksf

      if ( (__err = lua_pcall(__L, 0, 0, 0)) != 0 ) { // execute skill string
	// There was an error while executing the initialization file
	__errmsg = lua_tostring(__L, -1);
	lua_pop(__L, 1);
	switch (__err) {
	case LUA_ERRRUN:
	  logger->log_error("SkillerExecutionThread", "Lua runtime error: %s", __errmsg.c_str());
	  break;

	case LUA_ERRMEM:
	  logger->log_error("SkillerExecutionThread", "Lua ran out of memory");
	  break;

	case LUA_ERRERR:
	  logger->log_error("SkillerExecutionThread", "Lua runtime error and error function failed: %s", __errmsg.c_str());
	  break;
	}
      }

      publish_skill_status(curss);

      if ( ! __continuous_run ) {
	// was one-shot execution, cleanup
	logger->log_debug("SkillerExecutionThread", "Resetting skills");
	luaL_dostring(__L, "general.skillenv.reset_all()");
      }
    }
  } // end if (curss != "")
  __lua_mutex->unlock();
}


void
SkillerExecutionThread::fam_event(const char *filename, unsigned int mask)
{
  restart_lua();
}
