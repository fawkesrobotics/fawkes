
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

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/barrier.h>

extern "C" {
#include <lauxlib.h>
#include <lualib.h>
}

#include <string>

#define INIT_FILE SKILLDIR"/general/init.lua"

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
 */
SkillerExecutionThread::SkillerExecutionThread(Barrier *liaison_exec_barrier)
  : Thread("SkillerExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
  __liaison_exec_barrier = liaison_exec_barrier;
}


/** Destructor. */
SkillerExecutionThread::~SkillerExecutionThread()
{
}


void
SkillerExecutionThread::init()
{
  L = luaL_newstate();
  luaL_openlibs(L);

  lua_pushstring(L, SKILLDIR);
  lua_setglobal(L, "SKILLDIR");

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

  // initial loading of all skills
}


void
SkillerExecutionThread::finalize()
{
  lua_close(L);
}


void
SkillerExecutionThread::loop()
{
  __liaison_exec_barrier->wait();

  lua_pcall(L, 0, 0, 0);
}
