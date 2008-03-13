
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
#include <core/threading/mutex.h>
#include <core/threading/barrier.h>
#include <utils/logging/component.h>

#include <interfaces/object.h>

extern "C" {
#include <lauxlib.h>
#include <lualib.h>
#include <tolua++.h>
}

#include <string>
#include <cerrno>
#ifdef HAVE_INOTIFY
#  include <sys/inotify.h>
#  include <poll.h>
#endif

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
  __L = NULL;
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

  lua_pushstring(tL, SKILLDIR);  lua_setglobal(tL, "SKILLDIR");
  lua_pushstring(tL, LIBDIR);    lua_setglobal(tL, "LIBDIR");

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
  tolua_pushusertype(tL, config, "Configuration");
  lua_setglobal(tL, "config");

  tolua_pushusertype(tL, __clog, "ComponentLogger");
  lua_setglobal(tL, "logger");

  tolua_pushusertype(tL, clock, "Clock");
  lua_setglobal(tL, "clock");

  // Make sure Lua is not currently being executed
  __lua_mutex->lock();
  if ( __L != NULL ) {
    lua_close(__L);
  }
  __L = tL;
  __lua_mutex->unlock();
}


void
SkillerExecutionThread::start_lua()
{
  // Get interfaces from liaison thread
  tolua_pushusertype(__L, __slt->wm_ball_interface, __slt->wm_ball_interface->type());
  lua_setglobal(__L, "wm_ball_interface");


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


void
SkillerExecutionThread::init_inotify()
{
#ifdef HAVE_INOTIFY
  if ( (__inotify_fd = inotify_init()) == -1 ) {
    throw Exception(errno, "Failed to initialize inotify");
  }

  int regerr = 0;
  if ( (regerr = regcomp(&__inotify_regex, "^[^.].*\\.lua$", REG_EXTENDED)) != 0 ) {
    char errtmp[1024];
    regerror(regerr, &__inotify_regex, errtmp, sizeof(errtmp));
    regfree(&__inotify_regex);
    throw Exception("Failed to compile lua file regex: %s", errtmp);
  }

  // from http://www.linuxjournal.com/article/8478
  __inotify_bufsize = 1024 * (sizeof(struct inotify_event) + 16);
  __inotify_buf     = (char *)malloc(__inotify_bufsize);

  logger->log_debug("SkillerExecutionThread", "Adding watch for %s", SKILLDIR);
  uint32_t mask = IN_MODIFY | IN_MOVE | IN_CREATE | IN_DELETE;
  if ( (__inotify_skilldir_watch = inotify_add_watch(__inotify_fd, SKILLDIR, mask)) == -1) {
    close(__inotify_fd);
    throw Exception(errno, "Failed to add inotify watch for skilldir %s", SKILLDIR);
  }

#else
  logger->log_warn("SkillerExecutionThread", "inotify support not available, auto-reload "
		   "not available.");
#endif
}


void
SkillerExecutionThread::proc_inotify()
{
#ifdef HAVE_INOTIFY
  bool need_restart = false;

  // Check for inotify events
  pollfd ipfd;
  ipfd.fd = __inotify_fd;
  ipfd.events = POLLIN;
  ipfd.revents = 0;
  int prv = poll(&ipfd, 1, 0);
  if ( prv == -1 ) {
    logger->log_error("SkillerExecutionThread", "inotify poll failed: %s (%i)",
		      strerror(errno), errno);
  } else while ( prv > 0 ) {
    // Our fd has an event, we can read
    if ( ipfd.revents & POLLERR ) {      
      logger->log_error("SkillerExecutionThread", "inotify poll error");
    } else {
      // must be POLLIN
      int bytes = 0, i = 0;
      if ((bytes = read(__inotify_fd, __inotify_buf, __inotify_bufsize)) != -1) {
	while (i < bytes) {
	  struct inotify_event *event = (struct inotify_event *) &__inotify_buf[i];

	  if ( regexec(&__inotify_regex, event->name, 0, NULL, 0) == 0 ) {
	    // it is a *.lua file

	    if (event->mask & IN_MODIFY) {
	      logger->log_debug("SkillerExecutionThread", "File %s has been modified", event->name);
	    }
	    if (event->mask & IN_MOVE) {
	      logger->log_debug("SkillerExecutionThread", "File %s has been moved", event->name);
	    }
	    if (event->mask & IN_CREATE) {
	      logger->log_debug("SkillerExecutionThread", "File %s has been created", event->name);
	    }
	    if (event->mask & IN_DELETE) {
	      logger->log_debug("SkillerExecutionThread", "File %s has been deleted", event->name);
	    }

	    need_restart = true;
	  }

	  i += sizeof(struct inotify_event) + event->len;
	}
      } else {
	logger->log_error("SkillerExecutionThread", "inotify failed to read any bytes");
      }
    }

    prv = poll(&ipfd, 1, 0);
  }

  if ( need_restart ) {
    logger->log_info("SkillerExecutionThread", "inotify event triggers Lua restart");
    restart_lua();
  }

#else
  logger->log_error("SkillerExecutionThread", "inotify support not available, but "
		   "proc_inotify() was called. Ignoring.");
#endif
}


void
SkillerExecutionThread::init()
{
  init_inotify();

  __clog = new ComponentLogger(logger, "SkillerLua");
  __lua_mutex = new Mutex();

  try {
    init_lua();
  } catch (Exception &e) {
    delete __clog;
    delete __lua_mutex;
#ifdef HAVE_INOTIFY
    close(__inotify_fd);
#endif
    throw;
  }
}


void
SkillerExecutionThread::finalize()
{
  lua_close(__L);
  delete __clog;
  __clog = NULL;
  delete __lua_mutex;
  __lua_mutex = NULL;
#ifdef HAVE_INOTIFY
  inotify_rm_watch(__inotify_fd, __inotify_skilldir_watch);
  close(__inotify_fd);
  free(__inotify_buf);
  regfree(&__inotify_regex);
#endif
}


void
SkillerExecutionThread::once()
{
  start_lua();
}


void
SkillerExecutionThread::loop()
{
#ifdef HAVE_INOTIFY
  proc_inotify();
#endif

  __liaison_exec_barrier->wait();

  __lua_mutex->lock();
  //luaL_dostring(__L, "print(\"Ball X: \" .. wm_ball_interface:world_x());");
  // lua_pcall(__L, 0, 0, 0);
  __lua_mutex->unlock();
}
