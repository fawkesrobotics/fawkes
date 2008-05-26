
/***************************************************************************
 *  context.cpp - Fawkes Lua Context
 *
 *  Created: Fri May 23 15:53:54 2008
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

#include <lua/context.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exceptions/system.h>
#include <core/exceptions/software.h>

#include <tolua++.h>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace fawkes {

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
 * @param watch_dirs true to watch added package and C package dirs for
 * changes
 */
LuaContext::LuaContext(bool watch_dirs)
{
  if ( watch_dirs ) {
    __fam = new FileAlterationMonitor();
    __fam->add_filter("^[^.].*\\.lua$"); 
    __fam->add_listener(this);
  } else {
    __fam = NULL;
  }
  __lua_mutex = new Mutex();

  __start_script = NULL;
  __L = init_state();
}


/** Destructor. */
LuaContext::~LuaContext()
{
  __lua_mutex->lock();
  delete __fam;
  delete __lua_mutex;
  if ( __start_script )  free(__start_script);
  lua_close(__L);
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

  // Add package paths
  for (__slit = __package_dirs.begin(); __slit != __package_dirs.end(); ++__slit) {
    char *s;
    asprintf(&s, "package.path = package.path .. \";\" .. \"%s\" .. \"/?.lua\"", __slit->c_str());
    do_string(L, s);
    free(s);
  }

  for (__slit = __cpackage_dirs.begin(); __slit != __cpackage_dirs.end(); ++__slit) {
    char *s;
    asprintf(&s, "package.cpath = package.cpath .. \";\" .. \"%s\" .. \"/?.so\"", __slit->c_str());
    do_string(L, s);
    free(s);
  }

  // load base packages
  for (__slit = __packages.begin(); __slit != __packages.end(); ++__slit) {
    char *s;
    asprintf(&s, "require(\"%s\")", __slit->c_str());
    do_string(L, s);
    free(s);
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

  if ( __start_script ) {
    if (access(__start_script, R_OK) == 0) {
      // it's a file and we can access it, execute it!
      do_file(L, __start_script);
    } else {
      do_string(L, "require(\"%s\")", __start_script);
    }
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
  lua_State *L = init_state();
  lua_State *tL = __L;
  __L = L;
  lua_close(tL);
}


/** Add a Lua package directory.
 * The directory is added to the search path for lua packages. Files with
 * a .lua suffix will be considered as Lua modules.
 * @param path path to add
 */
void
LuaContext::add_package_dir(const char *path)
{
  MutexLocker lock(__lua_mutex);

  char *s;
  asprintf(&s, "package.path = package.path .. \";\" .. \"%s\" .. \"/?.lua\"", path);
  do_string(__L, s);
  free(s);

  __package_dirs.push_back(path);
  if ( __fam )  __fam->watch_dir(path);
}


/** Add a Lua C package directory.
 * The directory is added to the search path for lua C packages. Files
 * with a .so suffix will be considered as Lua modules.
 * @param path path to add
 */
void
LuaContext::add_cpackage_dir(const char *path)
{
  MutexLocker lock(__lua_mutex);

  char *s;
  asprintf(&s, "package.cpath = package.cpath .. \";\" .. \"%s\" .. \"/?.so\"", path);
  do_string(__L, s);
  free(s);

  __cpackage_dirs.push_back(path);
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

  char *s;
  asprintf(&s, "require(\"%s\")", package);
  do_string(__L, s);
  free(s);

  __packages.push_back(package);
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

  if ( (err = lua_pcall(L, 0, 0, 0)) != 0 ) {
    // There was an error while executing the initialization file
    errmsg = lua_tostring(L, -1);
    lua_pop(L, 1);
    switch (err) {
    case LUA_ERRRUN:
      throw Exception("Lua runtime error: %s", errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not execute Lua file %s", filename);

    case LUA_ERRERR:
      throw Exception("Failed to execute error handler during error: %s", errmsg.c_str());
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
  if ( luaL_dostring(L, s) != 0 ) {
    throw Exception(lua_tostring(L, -1));
  }
  free(s);
  va_end(arg);
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
  if ( luaL_dostring(__L, s) != 0 ) {
    throw Exception(lua_tostring(__L, -1));
  }
  free(s);
  va_end(arg);
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
  if ( (err = lua_pcall(__L, nargs, nresults, errfunc)) != 0 ) {
    // There was an error while executing the initialization file
    std::string errmsg = lua_tostring(__L, -1);
    lua_pop(__L, 1);
    switch (err) {
    case LUA_ERRRUN:
      throw Exception("Lua runtime error (pcall): %s", errmsg.c_str());

    case LUA_ERRMEM:
      throw OutOfMemoryException("Could not execute Lua chunk via pcall");

    case LUA_ERRERR:
      throw Exception("Failed to execute error handler during error (pcall): %s", errmsg.c_str());
    }
  }
}


/** Assert that the name is unique.
 * Checks the internal context structures if the name has been used
 * already.
 * @param name name to check
 */
void
LuaContext::assert_unique_name(const char *name)
{
  if ( __usertypes.find(name) != __usertypes.end() ) {
    throw Exception("User type entry already exists for name %s", name);
  }
  if ( __strings.find(name) != __strings.end() ) {
    throw Exception("String entry already exists for name %s", name);
  }
  if ( __booleans.find(name) != __booleans.end() ) {
    throw Exception("Boolean entry already exists for name %s", name);
  }
  if ( __numbers.find(name) != __numbers.end() ) {
    throw Exception("Number entry already exists for name %s", name);
  }
  if ( __integers.find(name) != __integers.end() ) {
    throw Exception("Integer entry already exists for name %s", name);
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

  assert_unique_name(name);

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
  assert_unique_name(name);

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
  assert_unique_name(name);

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
  assert_unique_name(name);

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
  assert_unique_name(name);

  __integers[name] = value;

  lua_pushinteger(__L, value);
  lua_setglobal(__L, name);
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


/** Push string on top of stack.
 * @param value value to push
 */
void
LuaContext::push_string(const char *value)
{
  MutexLocker lock(__lua_mutex);
  lua_pushstring(__L, value);
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


/** Push number on top of stack.
 * @param value value to push
 */
void
LuaContext::push_number(lua_Number value)
{
  MutexLocker lock(__lua_mutex);
  lua_pushnumber(__L, value);
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


/** Pop value(s) from stack.
 * @param n number of values to pop
 */
void
LuaContext::pop(int n)
{
  MutexLocker lock(__lua_mutex);
  lua_pop(__L, n);
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

  lua_pushnil(__L);
  lua_setglobal(__L, name);
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


/** Check if stack value is a number.
 * @param idx stack index of value
 * @return true if value is a number
 */
bool
LuaContext::is_number(int idx)
{
  return lua_isnumber(__L, idx);
}


/** Check if stack value is a boolean.
 * @param idx stack index of value
 * @return true if value is a boolean
 */
bool
LuaContext::is_boolean(int idx)
{
  return lua_isboolean(__L, idx);
}


/** Check if stack value is a string.
 * @param idx stack index of value
 * @return true if value is a string
 */
bool
LuaContext::is_string(int idx)
{
  return lua_isstring(__L, idx);
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
  lua_setfenv(__L, idx);
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
