
/***************************************************************************
 *  context.cpp - Fawkes Lua Context
 *
 *  Created: Fri May 23 15:53:54 2008
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <lua/context.h>
#include <lua/context_watcher.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exceptions/system.h>
#include <core/exceptions/software.h>
#include <logging/liblogger.h>

#include <algorithm>
#include <tolua++.h>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LuaContext <lua/context.h>
 * Lua C++ wrapper.
 * This thin wrapper allows for easy integration of Fawkes into other
 * applications. It provides convenience methods to some Lua and
 * tolua++ features like setting global variables or pushing/popping
 * values.
 *
 * It allows raw access to the Lua state since this class does not and
 * should not provide all the features Lua provides. If you use this
 * make sure that you lock the Lua context to avoid multi-threading
 * problems (if that is a possible concern in your application).
 *
 * LuaContext can use a FileAlterationMonitor on all added package and
 * C package directories. If anything changes in these directories the
 * Lua instance is then automatically restarted (closed, re-opened and
 * re-initialized).
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param enable_tracebacks if true an error function is installed at the top
 * of the stackand used for pcalls where errfunc is 0.
 */
LuaContext::LuaContext(bool enable_tracebacks)
{
  __owns_L = true;
  __enable_tracebacks = enable_tracebacks;
  __fam = NULL;
  __fam_thread = NULL;

  __lua_mutex = new Mutex();

  __start_script = NULL;
  __L = init_state();
}

/** Wrapper contstructor.
 * This wraps around an existing Lua state. It does not initialize the state in
 * the sense that it would add variables etc. It only provides convenient access
 * to the state methods via a C++ interface. It's mainly intended to be used to
 * create a LuaContext to be passed to LuaContextWatcher::lua_restarted(). The
 * state is not closed on destruction as is done when using the other ctor.
 * @param L Lua state to wrap
 */
LuaContext::LuaContext(lua_State *L)
{
  __owns_L = false;
  __L = L;
  __lua_mutex = new Mutex();
  __start_script = NULL;
  __fam = NULL;
  __fam_thread = NULL;
}

/** Destructor. */
LuaContext::~LuaContext()
{
  __lua_mutex->lock();

  if (! __finalize_call.empty())
	  do_string(__L, "%s", __finalize_call.c_str());

  if (__fam_thread) {
    __fam_thread->cancel();
    __fam_thread->join();
    delete __fam_thread;
  }
  delete __lua_mutex;
  if ( __start_script )  free(__start_script);
  if ( __owns_L) {
    lua_close(__L);
  }
}


/** Setup file alteration monitor.
 * Setup an internal file alteration monitor that can react to changes
 * on Lua files and packages.
 * @param auto_restart automatically restart the Lua context in case
 * of an event
 * @param conc_thread true to run a concurrent thread for event
 * processing. If and only if you set this to false, ensure that
 * process_fam_events() periodically.
 */
void
LuaContext::setup_fam(bool auto_restart, bool conc_thread)
{
  __fam = new FileAlterationMonitor();
  __fam->add_filter("^[^.].*\\.lua$"); 
  if (auto_restart) {
    __fam->add_listener(this);
  }
  if (conc_thread) {
    __fam_thread = new FamThread(__fam);
    __fam_thread->start();
  }
}


/** Get file alteration monitor.
 * @return reference counted pointer to file alteration monitor. Note
 * that the pointer might be invalid if setup_fam() has not been called.
 */
RefPtr<FileAlterationMonitor>
LuaContext::get_fam() const
{
  return __fam;
}


/** Initialize Lua state.
 * Initializes the state and makes all necessary initializations.
 * @return fresh initialized Lua state
 */
lua_State *
LuaContext::init_state()
{
  lua_State *L = luaL_newstate();
  luaL_openlibs(L);

  if (__enable_tracebacks) {
    lua_getglobal(L, "debug");
    lua_getfield(L, -1, "traceback");
    lua_remove(L, -2);
  }

  // Add package paths
  for (__slit = __package_dirs.begin(); __slit != __package_dirs.end(); ++__slit) {
    do_string(L, "package.path = package.path .. \";%s/?.lua;%s/?/init.lua\"", __slit->c_str(), __slit->c_str());
  }

  for (__slit = __cpackage_dirs.begin(); __slit != __cpackage_dirs.end(); ++__slit) {
    do_string(L, "package.cpath = package.cpath .. \";%s/?.so\"", __slit->c_str());
  }

  // load base packages
  for (__slit = __packages.begin(); __slit != __packages.end(); ++__slit) {
    do_string(L, "require(\"%s\")", __slit->c_str());
  }

  for ( __utit = __usertypes.begin(); __utit != __usertypes.end(); ++__utit) {
    tolua_pushusertype(L, __utit->second.first, __utit->second.second.c_str());
    lua_setglobal(L, __utit->first.c_str());
  }

  for ( __strings_it = __strings.begin(); __strings_it != __strings.end(); ++__strings_it) {
    lua_pushstring(L, __strings_it->second.c_str());
    lua_setglobal(L, __strings_it->first.c_str());
  }

  for ( __booleans_it = __booleans.begin(); __booleans_it != __booleans.end(); ++__booleans_it) {
    lua_pushboolean(L, __booleans_it->second);
    lua_setglobal(L, __booleans_it->first.c_str());
  }

  for ( __numbers_it = __numbers.begin(); __numbers_it != __numbers.end(); ++__numbers_it) {
    lua_pushnumber(L, __numbers_it->second);
    lua_setglobal(L, __numbers_it->first.c_str());
  }

  for ( __integers_it = __integers.begin(); __integers_it != __integers.end(); ++__integers_it) {
    lua_pushinteger(L, __integers_it->second);
    lua_setglobal(L, __integers_it->first.c_str());
  }

  for ( __cfuncs_it = __cfuncs.begin(); __cfuncs_it != __cfuncs.end(); ++__cfuncs_it)
  {
    lua_pushcfunction(L, __cfuncs_it->second);
    lua_setglobal(L, __cfuncs_it->first.c_str());
  }

  LuaContext *tmpctx = new LuaContext(L);

  MutexLocker(__watchers.mutex());
  LockList<LuaContextWatcher *>::iterator i;
  for (i = __watchers.begin(); i != __watchers.end(); ++i) {
	  try {
		  (*i)->lua_restarted(tmpctx);
	  } catch (...) {
		  try {
			  if (! __finalize_call.empty())
				  do_string(L, "%s", __finalize_call.c_str());
		  } catch (Exception &e) {} // ignored
		  
		  delete tmpctx;
      lua_close(L);
      throw;
    }
  }
  delete tmpctx;

  try {
	  if ( __start_script ) {
		  if (access(__start_script, R_OK) == 0) {
			  // it's a file and we can access it, execute it!
			  do_file(L, __start_script);
		  } else {
			  do_string(L, "require(\"%s\")", __start_script);
		  }
	  }
  } catch (...) {
	  if (! __finalize_call.empty())
		  do_string(L, "%s", __finalize_call.c_str());
	  lua_close(L);
    throw;
  }

  return L;
}


/** Set start script.
 * The script will be executed once immediately in this method, make
 * sure you call this after all other init-relevant routines like
 * add_* if you need to access these in the start script!
 * @param start_script script to execute now and on restart(). If the
 * string is the path and name of an accessible file it is loaded via
 * do_file(), otherwise it is considered to be the name of a module and
 * loaded via Lua's require(). Note however, that if you use a module,
 * special care has to be taken to correctly modify the global
 * environment!
 */
void
LuaContext::set_start_script(const char *start_script)
{
  if ( __start_script )  free(__start_script);
  if ( start_script ) {
    __start_script = strdup(start_script);
    if (access(__start_script, R_OK) == 0) {
      // it's a file and we can access it, execute it!
      do_file(__start_script);
    } else {
      do_string("require(\"%s\")", __start_script);
    }
  } else {
    __start_script = NULL;
  }
}


/** Restart Lua.
 * Creates a new Lua state, initializes it, anf if this went well the
 * current state is swapped with the new state.
 */
void
LuaContext::restart()
{
  MutexLocker lock(__lua_mutex);
  try {
	  if (! __finalize_prepare_call.empty())
		  do_string(__L, "%s", __finalize_prepare_call.c_str());
	  
	  lock.unlock();
	  lua_State *L = init_state();
	  lock.relock();
    lua_State *tL = __L;

    try {
	    if (! __finalize_call.empty())
		    do_string(__L, "%s", __finalize_call.c_str());
    } catch (Exception &e) {
	    LibLogger::log_warn("LuaContext", "Finalization call on old context failed, "
	                        "exception follows, ignoring.");
	    LibLogger::log_warn("LuaContext", e);
    }

    __L = L;
    if (__owns_L)  lua_close(tL);
    __owns_L = true;

  } catch (Exception &e) {
    LibLogger::log_error("LuaContext", "Could not restart Lua instance, an error "
			 "occured while initializing new state. Keeping old state.");
    LibLogger::log_error("LuaContext", e);
    if (! __finalize_cancel_call.empty())
	    do_string(__L, "%s", __finalize_cancel_call.c_str());
  }
}


/** Add a Lua package directory.
 * The directory is added to the search path for lua packages. Files with
 * a .lua suffix will be considered as Lua modules.
 * @param path path to add
 * @param prefix if true, insert path at the beginning of the search path,
 * append to end otherwise
 */
void
LuaContext::add_package_dir(const char *path, bool prefix)
{
  MutexLocker lock(__lua_mutex);

  if (prefix) {
    do_string(__L,
	      "package.path = \"%s/?.lua;%s/?/init.lua;\""
	      ".. package.path", path, path);

    __package_dirs.push_front(path);
  } else {
    do_string(__L,
	      "package.path = package.path .. "
	      "\";%s/?.lua;%s/?/init.lua\"", path, path);

    __package_dirs.push_back(path);
  }
  if ( __fam )  __fam->watch_dir(path);
}


/** Add a Lua C package directory.
 * The directory is added to the search path for lua C packages. Files
 * with a .so suffix will be considered as Lua modules.
 * @param path path to add
 * @param prefix if true, insert path at the beginning of the search path,
 * append to end otherwise
 */
void
LuaContext::add_cpackage_dir(const char *path, bool prefix)
{
  MutexLocker lock(__lua_mutex);

  if (prefix) {
    do_string(__L, "package.cpath = \"%s/?.so;\" .. package.cpath", path);

    __cpackage_dirs.push_front(path);
  } else {
    do_string(__L, "package.cpath = package.cpath .. \";%s/?.so\"", path);

    __cpackage_dirs.push_back(path);
  }
  if ( __fam )  __fam->watch_dir(path);
}


/** Add a default package.
 * Packages that are added this way are automatically loaded now and
 * on restart().
 * @param package package to add
 */
void
LuaContext::add_package(const char *package)
{
  MutexLocker lock(__lua_mutex);
  if (find(__packages.begin(), __packages.end(), package) == __packages.end()) {
    do_string(__L, "require(\"%s\")", package);

    __packages.push_back(package);
  }
}


/** Get Lua state.
 * Allows for raw modification of the used Lua state. Remember proper
 * locking!
 * @return Currently used Lua state
 */
lua_State *
LuaContext::get_lua_state()
{
  return __L;
}


/** Lock Lua state. */
void
LuaContext::lock()
{
  __lua_mutex->lock();
}


/** Try to lock the Lua state.
 * @return true if the state has been locked, false otherwise.
 */
bool
LuaContext::try_lock()
{
  return __lua_mutex->try_lock();
}


/** Unlock Lua state. */
void
LuaContext::unlock()
{
  __lua_mutex->unlock();
}


/** Execute file.
 * @param filename filet to load and excute.
 */
void
LuaContext::do_file(const char *filename)
{
  MutexLocker lock(__lua_mutex);
  do_file(__L, filename);
}


/** Execute file on a specific Lua state.
 * @param L Lua state to execute the file in.
 * @param filename filet to load and excute.
 */
void
LuaContext::do_file(lua_State *L, const char *filename)
{
  // Load initialization code
  int err = 0;
  std::string errmsg;
  if ( (err = luaL_loadfile(L, filename)) != 0) {
    errmsg = lua_tostring(L, -1);
    lua_pop(L, 1);
    switch (err) {
    case LUA_ERRSYNTAX:
      throw SyntaxErrorException("Lua syntax error in file %s: %s", filename, errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not load Lua file %s", filename);

    case LUA_ERRFILE:
      throw CouldNotOpenFileException(filename, errmsg.c_str());
    }
  }

  int errfunc = __enable_tracebacks ? 1 : 0;
  if ( (err = lua_pcall(L, 0, LUA_MULTRET, errfunc)) != 0 ) {
    // There was an error while executing the initialization file
    errmsg = lua_tostring(L, -1);
    lua_pop(L, 1);
    switch (err) {
    case LUA_ERRRUN:
      throw LuaRuntimeException("do_file", errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not execute Lua file %s", filename);

    case LUA_ERRERR:
      throw LuaErrorException("do_file", errmsg.c_str());

    default:
      throw LuaErrorException("do_file/unknown error", errmsg.c_str());
    }
  }

}


/** Execute string on a specific Lua state.
 * @param L Lua state to execute the string in
 * @param format format of string to execute, arguments can be the same as
 * for vasprintf.
 */
void
LuaContext::do_string(lua_State *L, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  char *s;
  if (vasprintf(&s, format, arg) == -1) {
    throw Exception("LuaContext::do_string: Could not form string");
  }
  std::string ss(s);
  free(s);
  va_end(arg);

  int err = 0;
  std::string errmsg;
  if ( (err = luaL_loadstring(L, ss.c_str())) != 0) {
    errmsg = lua_tostring(L, -1);
    lua_pop(L, 1);
    switch (err) {
    case LUA_ERRSYNTAX:
	    throw SyntaxErrorException("Lua syntax error in string %s: %s", ss.c_str(), errmsg.c_str());

    case LUA_ERRMEM:
	    throw OutOfMemoryException("Could not load Lua string %s", ss.c_str());
    }
  }

  int errfunc = __enable_tracebacks ? 1 : 0;
  err = lua_pcall(L, 0, LUA_MULTRET, errfunc);

  if (err != 0) {
	  std::string errmsg = lua_tostring(L, -1);
	  lua_pop(L, 1);
	  switch (err) {
	  case LUA_ERRRUN:
		  throw LuaRuntimeException("do_string", errmsg.c_str());

	  case LUA_ERRMEM:
		  throw OutOfMemoryException("Could not execute Lua chunk via pcall");

	  case LUA_ERRERR:
		  throw LuaErrorException("do_string", errmsg.c_str());
	  }
  }
}


/** Execute string.
 * @param format format of string to execute, arguments can be the same as
 * for vasprintf.
 */
void
LuaContext::do_string(const char *format, ...)
{
  MutexLocker lock(__lua_mutex);

  va_list arg;
  va_start(arg, format);
  char *s;
  if (vasprintf(&s, format, arg) == -1) {
    throw Exception("LuaContext::do_string: Could not form string");
  }
  std::string ss(s);
  free(s);
  va_end(arg);

  int err = 0;
  std::string errmsg;
  if ( (err = luaL_loadstring(__L, ss.c_str())) != 0) {
    errmsg = lua_tostring(__L, -1);
    lua_pop(__L, 1);
    switch (err) {
    case LUA_ERRSYNTAX:
	    throw SyntaxErrorException("Lua syntax error in string %s: %s", ss.c_str(), errmsg.c_str());

    case LUA_ERRMEM:
	    throw OutOfMemoryException("Could not load Lua string %s", ss.c_str());
    }
  }

  int errfunc = __enable_tracebacks ? 1 : 0;
  err = lua_pcall(__L, 0, LUA_MULTRET, errfunc);

  if (err != 0) {
	  std::string errmsg = lua_tostring(__L, -1);
	  lua_pop(__L, 1);
	  switch (err) {
	  case LUA_ERRRUN:
		  throw LuaRuntimeException("do_string", errmsg.c_str());

	  case LUA_ERRMEM:
		  throw OutOfMemoryException("Could not execute Lua chunk via pcall");

	  case LUA_ERRERR:
		  throw LuaErrorException("do_string", errmsg.c_str());
	  }
  }
}


/** Load Lua string.
 * Loads the Lua string and places it as a function on top of the stack.
 * @param s string to load
 */
void
LuaContext::load_string(const char *s)
{
  int err;
  if ( (err = luaL_loadstring(__L, s)) != 0 ) {
    std::string errmsg = lua_tostring(__L, -1);
    lua_pop(__L, 1);
    switch (err) {
    case LUA_ERRSYNTAX:
      throw SyntaxErrorException("Lua syntax error in string '%s': %s",
				 s, errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not load Lua string '%s'", s);
    }
  }
}


/** Protected call.
 * Calls the function on top of the stack. Errors are handled gracefully.
 * @param nargs number of arguments
 * @param nresults number of results
 * @param errfunc stack index of an error handling function
 * @exception Exception thrown for generic runtime error or if the
 * error function could not be executed.
 * @exception OutOfMemoryException thrown if not enough memory was available
 */
void
LuaContext::pcall(int nargs, int nresults, int errfunc)
{
  int err = 0;
  if ( ! errfunc && __enable_tracebacks )  errfunc = 1;
  if ( (err = lua_pcall(__L, nargs, nresults, errfunc)) != 0 ) {
    std::string errmsg = lua_tostring(__L, -1);
    lua_pop(__L, 1);
    switch (err) {
    case LUA_ERRRUN:
      throw LuaRuntimeException("pcall", errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not execute Lua chunk via pcall");

    case LUA_ERRERR:
      throw LuaErrorException("pcall", errmsg.c_str());
    }
  }
}


/** Assert that the name is unique.
 * Checks the internal context structures if the name has been used
 * already. It will accept a value that has already been set that is of the same
 * type as the one supplied. Pass the empty string to avoid this.
 * @param name name to check
 * @param type type of value
 * @exception Exception thrown if name is not unique
 */
void
LuaContext::assert_unique_name(const char *name, std::string type)
{
  if ( (type == "usertype") && (__usertypes.find(name) != __usertypes.end()) ) {
    throw Exception("User type entry already exists for name %s", name);
  }
  if ( (type == "string") && (__strings.find(name) != __strings.end()) ) {
    throw Exception("String entry already exists for name %s", name);
  }
  if ( (type == "boolean") && (__booleans.find(name) != __booleans.end()) ) {
    throw Exception("Boolean entry already exists for name %s", name);
  }
  if ( (type == "number") && (__numbers.find(name) != __numbers.end()) ) {
    throw Exception("Number entry already exists for name %s", name);
  }
  if ( (type == "integer") && (__integers.find(name) != __integers.end()) ) {
    throw Exception("Integer entry already exists for name %s", name);
  }
  if ( (type == "cfunction") && (__cfuncs.find(name) != __cfuncs.end()) ) {
    throw Exception("C function entry already exists for name %s", name);
  }
}


/** Assign usertype to global variable.
 * @param name name of global variable to assign the value to
 * @param data usertype data
 * @param type_name type name of the data
 * @param name_space C++ namespace of type, prepended to type_name
 */
void
LuaContext::set_usertype(const char *name, void *data,
			  const char *type_name, const char *name_space)
{
  MutexLocker lock(__lua_mutex);

  std::string type_n = type_name;
  if ( name_space ) {
    type_n = std::string(name_space) + "::" + type_name;
  }

  assert_unique_name(name, "usertype");

  __usertypes[name] = std::make_pair(data, type_n);

  tolua_pushusertype(__L, data, type_n.c_str());
  lua_setglobal(__L, name);
}


/** Assign string to global variable.
 * @param name name of global variable to assign the value to
 * @param value value to assign
 */
void
LuaContext::set_string(const char *name, const char *value)
{
  MutexLocker lock(__lua_mutex);
  assert_unique_name(name, "string");

  __strings[name] = value;

  lua_pushstring(__L, value);
  lua_setglobal(__L, name);
}


/** Assign boolean to global variable.
 * @param name name of global variable to assign the value to
 * @param value value to assign
 */
void
LuaContext::set_boolean(const char *name, bool value)
{
  MutexLocker lock(__lua_mutex);
  assert_unique_name(name, "boolean");

  __booleans[name] = value;

  lua_pushboolean(__L, value ? 1 : 0);
  lua_setglobal(__L, name);
}


/** Assign number to global variable.
 * @param name name of global variable to assign the value to
 * @param value value to assign
 */
void
LuaContext::set_number(const char *name, lua_Number value)
{
  MutexLocker lock(__lua_mutex);
  assert_unique_name(name, "number");

  __numbers[name] = value;

  lua_pushnumber(__L, value);
  lua_setglobal(__L, name);
}


/** Assign integer to global variable.
 * @param name name of global variable to assign the value to
 * @param value value to assign
 */
void
LuaContext::set_integer(const char *name, lua_Integer value)
{
  MutexLocker lock(__lua_mutex);
  assert_unique_name(name, "integer");

  __integers[name] = value;

  lua_pushinteger(__L, value);
  lua_setglobal(__L, name);
}


/** Assign cfunction to global variable.
 * @param name name of global variable to assign the value to
 * @param f function
 */
void
LuaContext::set_cfunction(const char *name, lua_CFunction f)
{
  MutexLocker lock(__lua_mutex);
  assert_unique_name(name, "cfunction");

  __cfuncs[name] = f;

  lua_pushcfunction(__L, f);
  lua_setglobal(__L, name);
}


/** Push boolean on top of stack.
 * @param value value to push
 */
void
LuaContext::push_boolean(bool value)
{
  MutexLocker lock(__lua_mutex);
  lua_pushboolean(__L, value ? 1 : 0);
}


/** Push formatted string on top of stack.
 * @param format string format
 * @see man 3 sprintf
 */
void
LuaContext::push_fstring(const char *format, ...)
{
  MutexLocker lock(__lua_mutex);
  va_list arg;
  va_start(arg, format);
  lua_pushvfstring(__L, format, arg);
  va_end(arg);
}


/** Push integer on top of stack.
 * @param value value to push
 */
void
LuaContext::push_integer(lua_Integer value)
{
  MutexLocker lock(__lua_mutex);
  lua_pushinteger(__L, value);
}


/** Push light user data on top of stack.
 * @param p pointer to light user data to push
 */
void
LuaContext::push_light_user_data(void *p)
{
  MutexLocker lock(__lua_mutex);
  lua_pushlightuserdata(__L, p);
}


/** Push substring on top of stack.
 * @param s string to push
 * @param len length of string to push
 */
void
LuaContext::push_lstring(const char *s, size_t len)
{
  MutexLocker lock(__lua_mutex);
  lua_pushlstring(__L, s, len);
}


/** Push nil on top of stack.
 */
void
LuaContext::push_nil()
{
  MutexLocker lock(__lua_mutex);
  lua_pushnil(__L);
}


/** Push number on top of stack.
 * @param value value to push
 */
void
LuaContext::push_number(lua_Number value)
{
  MutexLocker lock(__lua_mutex);
  lua_pushnumber(__L, value);
}


/** Push string on top of stack.
 * @param value value to push
 */
void
LuaContext::push_string(const char *value)
{
  MutexLocker lock(__lua_mutex);
  lua_pushstring(__L, value);
}


/** Push thread on top of stack.
 */
void
LuaContext::push_thread()
{
  MutexLocker lock(__lua_mutex);
  lua_pushthread(__L);
}


/** Push a copy of the element at the given index on top of the stack.
 * @param idx index of the value to copy
 */
void
LuaContext::push_value(int idx)
{
  MutexLocker lock(__lua_mutex);
  lua_pushvalue(__L, idx);
}


/** Push formatted string on top of stack.
 * @param format string format
 * @param arg variadic argument list
 * @see man 3 sprintf
 */
void
LuaContext::push_vfstring(const char *format, va_list arg)
{
  MutexLocker lock(__lua_mutex);
  lua_pushvfstring(__L, format, arg);
}


/** Push usertype on top of stack.
 * @param data usertype data
 * @param type_name type name of the data
 * @param name_space C++ namespace of type, prepended to type_name
 */
void
LuaContext::push_usertype(void *data, const char *type_name,
			  const char *name_space)
{
  MutexLocker lock(__lua_mutex);

  std::string type_n = type_name;
  if ( name_space ) {
    type_n = std::string(name_space) + "::" + type_name;
  }

  tolua_pushusertype(__L, data, type_n.c_str());
}


/** Push C function on top of stack.
 * @param f C Function to push
 */
void
LuaContext::push_cfunction(lua_CFunction f)
{
  MutexLocker lock(__lua_mutex);
  lua_pushcfunction(__L, f);
}


/** Get name of type of value at a given index.
 * @param idx index of value to get type for
 * @return name of type of the value at the given index
 */
std::string
LuaContext::type_name(int idx)
{
	return lua_typename(__L, lua_type(__L, idx));
}

/** Pop value(s) from stack.
 * @param n number of values to pop
 */
void
LuaContext::pop(int n)
{
  MutexLocker lock(__lua_mutex);
  if (__enable_tracebacks && (n >= stack_size())) {
    throw LuaRuntimeException("pop", "Cannot pop traceback function, invalid n");
  }
  lua_pop(__L, n);
}

/** Remove value from stack.
 * @param idx index of element to remove
 */
void
LuaContext::remove(int idx)
{
  MutexLocker lock(__lua_mutex);
  if (__enable_tracebacks && ((idx == 1) || (idx == -stack_size()))) {
    throw LuaRuntimeException("pop", "Cannot remove traceback function");
  }
  lua_remove(__L, idx);
}


/** Get size of stack.
 * @return number of elements on the stack
 */
int
LuaContext::stack_size()
{
  return lua_gettop(__L);
}


/** Create a table on top of the stack.
 * @param narr number of array elements
 * @param nrec number of non-array elements
 */
void
LuaContext::create_table(int narr, int nrec)
{
  lua_createtable(__L, narr, nrec);
}


/** Set value of a table.
 * Sets value t[k] = v. t is the table at the given index, by default it is the
 * third-last entry (index is -3). v is the value at the top of the stack, k
 * is the element just below the top.
 * @param t_index index of the table on the stack
 */
void
LuaContext::set_table(int t_index)
{
  lua_settable(__L, t_index);
}


/** Set field of a table.  Does the equivalent to t[k] = v, where t is
 * the value at the given valid index and v is the value at the top of
 * the stack.  This function pops the value from the stack. As in Lua,
 * this function may trigger a metamethod for the "newindex" event.
 * @param key key of the field to set @param t_index index of the
 * table on the stack, defaults to the element just below the value to
 * set (-2, second last element on the stack).
 */
void
LuaContext::set_field(const char *key, int t_index)
{
  lua_setfield(__L, t_index, key);
}


/** Set a global value.
 * Sets the global variable with the given name to the value currently on top
 * of the stack. No check whatsoever regarding the name is done.
 * @param name name of the variable to assign
 */
void
LuaContext::set_global(const char *name)
{
  lua_setglobal(__L, name);
}


/** Get value from table.
 * Assumes that an index k is at the top of the stack. Then t[k] is retrieved,
 * where t is a table at the given index idx. The resulting value is pushed
 * onto the stack, while the key k is popped from the stack, thus the value
 * replaces the key.
 * @param idx index of the table on the stack
 */
void
LuaContext::get_table(int idx)
{
  lua_gettable(__L, idx);
}


/** Get named value from table.
 * Retrieves the t[k], where k is the given key and t is a table at the given
 * index idx. The value is pushed onto the stack.
 * @param idx index of the table
 * @param k key of the table entry
 */
void
LuaContext::get_field(int idx, const char *k)
{
  lua_getfield(__L, idx, k);
}


/** Set value without invoking meta methods.
 * Similar to set_table(), but does raw access, i.e. without invoking meta-methods.
 * @param idx index of the table
 */
void
LuaContext::raw_set(int idx)
{
  lua_rawset(__L, idx);
}


/** Set indexed value without invoking meta methods.
 * Sets t[n]=v, where t is a table at index idx and v is the value at the
 * top of the stack.
 * @param idx index of the table
 * @param n index in the table
 */
void
LuaContext::raw_seti(int idx, int n)
{
  lua_rawseti(__L, idx, n);
}


/** Get value without invoking meta methods.
 * Similar to get_table(), but does raw access, i.e. without invoking meta-methods.
 * @param idx index of the table
 */
void
LuaContext::raw_get(int idx)
{
  lua_rawget(__L, idx);
}


/** Get indexed value without invoking meta methods.
 * Pushes t[n] onto the stack, where t is a table at index idx.
 * @param idx index of the table
 * @param n index in the table
 */
void
LuaContext::raw_geti(int idx, int n)
{
  lua_rawgeti(__L, idx, n);
}


/** Get global variable.
 * @param name name of the global variable
 */
void
LuaContext::get_global(const char *name)
{
  lua_getglobal(__L, name);
}


/** Remove global variable.
 * Assigns nil to the given variable and removes it from internal
 * assignment maps.
 * @param name name of value to remove
 */
void
LuaContext::remove_global(const char *name)
{
  MutexLocker lock(__lua_mutex);

  __usertypes.erase(name);
  __strings.erase(name);
  __booleans.erase(name);
  __numbers.erase(name);
  __integers.erase(name);
  __cfuncs.erase(name);

  lua_pushnil(__L);
  lua_setglobal(__L, name);
}


/** Iterate to next entry of table.
 * @param idx stack index of table
 * @return true if there was another iterable value in the table,
 * false otherwise
 */
bool
LuaContext::table_next(int idx)
{
	return lua_next(__L, idx) != 0;
}


/** Retrieve stack value as number.
 * @param idx stack index of value
 * @return value as number
 */
lua_Number
LuaContext::to_number(int idx)
{
  return lua_tonumber(__L, idx);
}


/** Retrieve stack value as integer.
 * @param idx stack index of value
 * @return value as integer
 */
lua_Integer
LuaContext::to_integer(int idx)
{
  return lua_tointeger(__L, idx);
}


/** Retrieve stack value as boolean.
 * @param idx stack index of value
 * @return value as boolean
 */
bool
LuaContext::to_boolean(int idx)
{
  return lua_toboolean(__L, idx);
}


/** Retrieve stack value as string.
 * @param idx stack index of value
 * @return value as string
 */
const char *
LuaContext::to_string(int idx)
{
  return lua_tostring(__L, idx);
}

/** Retrieve stack value as userdata.
 * @param idx stack index of value
 * @return value as userdata, maybe NULL
 */
void *
LuaContext::to_userdata(int idx)
{
  return lua_touserdata(__L, idx);
}


/** Retrieve stack value as pointer.
 * @param idx stack index of value
 * @return value as pointer, maybe NULL
 */
void *
LuaContext::to_pointer(int idx)
{
	return (void *)lua_topointer(__L, idx);
}


/** Retrieve stack value as a tolua++ user type.
 * @param idx stack index of value
 * @return value as pointer, maybe NULL
 */
void *
LuaContext::to_usertype(int idx)
{
	return tolua_tousertype(__L, idx, 0);
}


/** Check if stack value is a boolean.
 * @param idx stack index of value
 * @return true if value is a boolean, false otherwise
 */
bool
LuaContext::is_boolean(int idx)
{
  return lua_isboolean(__L, idx);
}


/** Check if stack value is a C function.
 * @param idx stack index of value
 * @return true if value is a C function, false otherwise
 */
bool
LuaContext::is_cfunction(int idx)
{
  return lua_iscfunction(__L, idx);
}


/** Check if stack value is a function.
 * @param idx stack index of value
 * @return true if value is a function, false otherwise
 */
bool
LuaContext::is_function(int idx)
{
  return lua_isfunction(__L, idx);
}


/** Check if stack value is light user data.
 * @param idx stack index of value
 * @return true if value is light user data , false otherwise
 */
bool
LuaContext::is_light_user_data(int idx)
{
  return lua_islightuserdata(__L, idx);
}


/** Check if stack value is nil.
 * @param idx stack index of value
 * @return true if value is nil, false otherwise
 */
bool
LuaContext::is_nil(int idx)
{
  return lua_isnil(__L, idx);
}


/** Check if stack value is a number.
 * @param idx stack index of value
 * @return true if value is a number, false otherwise
 */
bool
LuaContext::is_number(int idx)
{
  return lua_isnumber(__L, idx);
}


/** Check if stack value is a string.
 * @param idx stack index of value
 * @return true if value is a string, false otherwise
 */
bool
LuaContext::is_string(int idx)
{
  return lua_isstring(__L, idx);
}


/** Check if stack value is a table.
 * @param idx stack index of value
 * @return true if value is a table, false otherwise
 */
bool
LuaContext::is_table(int idx)
{
  return lua_istable(__L, idx);
}


/** Check if stack value is a thread.
 * @param idx stack index of value
 * @return true if value is a thread, false otherwise
 */
bool
LuaContext::is_thread(int idx)
{
  return lua_isthread(__L, idx);
}


/** Get object length
 * @param idx stack index of value
 * @return size of object
 */
size_t
LuaContext::objlen(int idx)
{
  return lua_objlen(__L, idx);
}


/** Set function environment.
 * Sets the table on top of the stack as environment of the function
 * at the given stack index.
 * @param idx stack index of function
 */
void
LuaContext::setfenv(int idx)
{
#if LUA_VERSION_NUM > 501
  //                                         stack: ... func@idx ... env
  // find _ENV
  int n = 0;
  const char *val_name;
  while ((val_name = lua_getupvalue(__L, idx, ++n)) != NULL) {  // ... env upval
    if (strcmp(val_name, "_ENV") == 0) {
      // found environment
      break;
    } else {
      // we're not interested, remove value from stack
      lua_pop(__L, 1);                                          // ... env
    }
  }

  // found an environment
  if (val_name != NULL) {                                       // ... env upval
    // create a new simple up value
    luaL_loadstring(__L, "");                                   // ... env upval chunk
    lua_pushvalue(__L, -3);                                     // ... env upval chunk env
    lua_setupvalue(__L, -2, 1);                                 // ... env upval func
    int act_idx = idx > 0 ? idx : idx - 2;
    lua_upvaluejoin(__L, act_idx, n, -1, 1);                        // ... env upval chunk
    lua_pop(__L, 3);                                            // ...
  } else {
    throw Exception("No environment found");
  }
#else
  lua_setfenv(__L, idx);
#endif
}

/** Add a context watcher.
 * @param watcher watcher to add
 */
void
LuaContext::add_watcher(fawkes::LuaContextWatcher *watcher)
{
  __watchers.push_back_locked(watcher);
}


/** Remove a context watcher.
 * @param watcher watcher to remove
 */
void
LuaContext::remove_watcher(fawkes::LuaContextWatcher *watcher)
{
  __watchers.remove_locked(watcher);
}


/** Set code to execute during finalization.
 * @param finalize code string to execute (via do_string()) when eventually
 * finalizing a context
 * @param finalize_prepare code string to execute (via do_string()) before
 * finalization is performed, for example during a context restart before the
 * new context is initialized
 * @param finalize_cancel code string to execute (via do_string()) if,
 * during a restart, the initialization of the new context failed and therefore
 * the previously prepared finalization must be cancelled
 */
void
LuaContext::set_finalization_calls(std::string finalize, std::string finalize_prepare,
                                   std::string finalize_cancel)
{
	__finalize_call = finalize;
	__finalize_prepare_call = finalize_prepare;
	__finalize_cancel_call = finalize_cancel;
}




/** Process FAM events. */
void
LuaContext::process_fam_events()
{
  if ( __fam)  __fam->process_events();
}


void
LuaContext::fam_event(const char *filename, unsigned int mask)
{
  restart();
}


} // end of namespace fawkes
