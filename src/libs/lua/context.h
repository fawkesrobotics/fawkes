
/***************************************************************************
 *  context.h - Fawkes Lua Context
 *
 *  Created: Fri May 23 11:29:01 2008
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

#ifndef _LUA_CONTEXT_H_
#define _LUA_CONTEXT_H_

#include <lua/exceptions.h>
#include <core/utils/lock_list.h>
#include <core/utils/refptr.h>
#include <utils/system/fam.h>
#include <utils/system/fam_thread.h>

#include <lua.hpp>

#include <map>
#include <utility>
#include <list>
#include <string>

namespace fawkes {

class LuaContextWatcher;
class Mutex;

class LuaContext : public FamListener
{
 public:
  LuaContext(bool enable_tracebacks = true);
  LuaContext(lua_State *L);
  ~LuaContext();

  void setup_fam(bool auto_restart, bool conc_thread);
  RefPtr<FileAlterationMonitor>  get_fam() const;

  void set_start_script(const char *start_script);
  void set_finalization_calls(std::string finalize,
                              std::string finalize_prepare,
                              std::string finalize_cancel);

  void restart();

  void add_package_dir(const char *path, bool prefix = false);
  void add_cpackage_dir(const char *path, bool prefix = false);
  void add_package(const char *package);

  lua_State *  get_lua_state();

  void lock();
  bool try_lock();
  void unlock();

  void do_file(const char *filename);
  void do_string(const char *format, ...);

  void load_string(const char *s);
  void pcall(int nargs = 0, int nresults = 0, int errfunc = 0);

  void set_usertype(const char *name, void *data, const char *type_name,
		     const char *name_space = 0);
  void set_string(const char *name, const char *value);
  void set_number(const char *name, lua_Number value);
  void set_boolean(const char *name, bool value);
  void set_integer(const char *name, lua_Integer value);
  void set_cfunction(const char *name, lua_CFunction f);
  void remove_global(const char *name);
  void set_global(const char *name);

  void push_boolean(bool value);
  void push_fstring(const char *format, ...);
  void push_integer(lua_Integer value);
  void push_light_user_data(void *p);
  void push_lstring(const char *s, size_t len);
  void push_nil();
  void push_number(lua_Number value);
  void push_string(const char *value);
  void push_thread();
  void push_value(int idx);
  void push_vfstring(const char *format, va_list arg);
  void push_usertype(void *data, const char *type_name,
		     const char *name_space = 0);
  void push_cfunction(lua_CFunction f);

  std::string type_name(int idx);

  void pop(int n);
  void remove(int idx);
  int  stack_size();

  void create_table(int narr = 0, int nrec = 0);
  void set_table(int t_index = -3);
  void set_field(const char *key, int t_index = -2);
  
  void get_table(int idx);
  void get_field(int idx, const char *k);
  void get_global(const char *name);

  bool table_next(int idx);
  
  void raw_set(int idx);
  void raw_seti(int idx, int n);
  void raw_get(int idx);
  void raw_geti(int idx, int n);

  lua_Number   to_number(int idx);
  lua_Integer  to_integer(int idx);
  bool         to_boolean(int idx);
  const char * to_string(int idx);
  void *       to_userdata(int idx);
  void *       to_pointer(int idx);
  void *       to_usertype(int idx);

  bool         is_boolean(int idx);
  bool         is_cfunction(int idx);
  bool         is_function(int idx);
  bool         is_light_user_data(int idx);
  bool         is_nil(int idx);
  bool         is_number(int idx);
  bool         is_string(int idx);
  bool         is_table(int idx);
  bool         is_thread(int idx);

  size_t       objlen(int idx);
  void         setfenv(int idx = -2);

  void         add_watcher(LuaContextWatcher *watcher);
  void         remove_watcher(LuaContextWatcher *watcher);

  /* from FamListener */
  virtual void fam_event(const char *filename, unsigned int mask);
  void process_fam_events();


 private:
  lua_State *  init_state();
  void         do_string(lua_State *L, const char *format, ...);
  void         do_file(lua_State *L, const char *s);
  void         assert_unique_name(const char *name, std::string type);

 
 private:
  lua_State *L_;
  bool       owns_L_;
  bool       enable_tracebacks_;

  Mutex  *lua_mutex_;
  char   *start_script_;

  std::list<std::string>            package_dirs_;
  std::list<std::string>            cpackage_dirs_;
  std::list<std::string>            packages_;
  std::list<std::string>::iterator  slit_;

  std::map<std::string, std::pair<void *, std::string> > usertypes_;
  std::map<std::string, std::pair<void *, std::string> >::iterator utit_;
  std::map<std::string, std::string>             strings_;
  std::map<std::string, std::string>::iterator   strings_it_;
  std::map<std::string, bool>                    booleans_;
  std::map<std::string, bool>::iterator          booleans_it_;
  std::map<std::string, lua_Number>              numbers_;
  std::map<std::string, lua_Number>::iterator    numbers_it_;
  std::map<std::string, lua_Integer>             integers_;
  std::map<std::string, lua_Integer>::iterator   integers_it_;
  std::map<std::string, lua_CFunction>           cfuncs_;
  std::map<std::string, lua_CFunction>::iterator cfuncs_it_;

  std::string finalize_call_;
  std::string finalize_prepare_call_;
  std::string finalize_cancel_call_;
  
  RefPtr<FileAlterationMonitor>  fam_;
  FamThread                     *fam_thread_;

  LockList<LuaContextWatcher *> watchers_;

};

} // end of namespace fawkes

#endif
