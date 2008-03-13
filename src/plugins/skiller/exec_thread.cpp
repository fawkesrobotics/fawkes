
/***************************************************************************
 *  exec_thread.cpp - Fawkes Skiller: Execution Thread
 *
 *  Created: Mon Feb 18 10:30:17 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/skiller/exec_thread.h>
#include <plugins/skiller/liaison_thread.h>

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/barrier.h>
#include <utils/logging/component.h>

#include <interfaces/object.h>

extern "C" {
#include <lauxlib.h>
#include <lualib.h>
#include <tolua++.h>
}

#include <string>

#define INIT_FILE SKILLDIR"/general/init.lua"
#define START_FILE SKILLDIR"/general/start.lua"

using namespace std;

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
}


/** Destructor. */
SkillerExecutionThread::~SkillerExecutionThread()
{
}


void
SkillerExecutionThread::init()
{
  __clog = new ComponentLogger(logger, "SkillerLua");

  L = luaL_newstate();
  luaL_openlibs(L);

  lua_pushstring(L, SKILLDIR);  lua_setglobal(L, "SKILLDIR");
  lua_pushstring(L, LIBDIR);    lua_setglobal(L, "LIBDIR");

  // Load initialization code
  if ( (err = luaL_loadfile(L, INIT_FILE)) != 0) {
    errmsg = lua_tostring(L, -1);
    lua_pop(L, 1);
    lua_close(L);
    switch (err) {
    case LUA_ERRSYNTAX:
      throw SyntaxErrorException("Lua syntax error: %s", errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not load Lua init file");

    case LUA_ERRFILE:
      throw CouldNotOpenFileException(INIT_FILE, errmsg.c_str());
    }
  }

  if ( (err = lua_pcall(L, 0, 0, 0)) != 0 ) {
    // There was an error while executing the initialization file
    errmsg = lua_tostring(L, -1);
    lua_pop(L, 1);
    lua_close(L);
    switch (err) {
    case LUA_ERRRUN:
      throw Exception("Lua runtime error: %s", errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not execute Lua init file");

    case LUA_ERRERR:
      throw Exception("Failed to execute error handler during error: %s", errmsg.c_str());
    }
  }

  // Export some utilities to Lua
  // NOTE: all the (Lua) types that you use here must have been declared before, probably
  // by having an appropriate require clause for a wrapper in init.lua!
  tolua_pushusertype(L, config, "Configuration");
  lua_setglobal(L, "config");

  tolua_pushusertype(L, __clog, "ComponentLogger");
  lua_setglobal(L, "logger");

  tolua_pushusertype(L, clock, "Clock");
  lua_setglobal(L, "clock");

  // initial loading of all skills
}


void
SkillerExecutionThread::finalize()
{
  lua_close(L);
  delete __clog;
  __clog = NULL;
}


void
SkillerExecutionThread::once()
{
  // Get interfaces from liaison thread
  tolua_pushusertype(L, __slt->wm_ball_interface, __slt->wm_ball_interface->type());
  lua_setglobal(L, "wm_ball_interface");


  // Load start code
  if ( (err = luaL_loadfile(L, START_FILE)) != 0) {
    errmsg = lua_tostring(L, -1);
    lua_pop(L, 1);
    switch (err) {
    case LUA_ERRSYNTAX:
      __clog->log_debug("Lua syntax error: %s", errmsg.c_str());
      break;

    case LUA_ERRMEM:
      __clog->log_debug("Lua: Out of memory, cannot load start file");
      break;

    case LUA_ERRFILE:
      __clog->log_debug("Lua: could not open start file (%s)", errmsg.c_str());
      break;

    default:
      __clog->log_debug("Lua: unknown error occured (%s)", errmsg.c_str());
      break;
    }
  } else {
    if ( (err = lua_pcall(L, 0, 0, 0)) != 0 ) {
      // There was an error while executing the initialization file
      errmsg = lua_tostring(L, -1);
      lua_pop(L, 1);
      switch (err) {
      case LUA_ERRRUN:
	__clog->log_debug("Lua runtime error: %s", errmsg.c_str());
	break;
	
      case LUA_ERRMEM:
	__clog->log_debug("Lua: Out of memory, cannot execute start file");
	break;

      case LUA_ERRERR:
	__clog->log_debug("Lua: Execution failed, error handled failed as well (%s)", errmsg.c_str());
	break;

      default:
	__clog->log_debug("Lua: unknown error occured (%s)", errmsg.c_str());
	break;
      }
    }
  }
}


void
SkillerExecutionThread::loop()
{
  __liaison_exec_barrier->wait();

  //luaL_dostring(L, "print(\"Ball X: \" .. wm_ball_interface:world_x());");
  // lua_pcall(L, 0, 0, 0);
}
