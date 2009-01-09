
/***************************************************************************
 *  interfaceimporter.h - Fawkes Lua Interface Importer
 *
 *  Created: Thu Jan 01 14:28:47 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __LUA_INTERFACEIMPORTER_H_
#define __LUA_INTERFACEIMPORTER_H_

#include <lua/context_watcher.h>

#include <core/utils/lock_map.h>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BlackBoard;
class Configuration;
class Interface;
class Logger;
class LuaContext;

class LuaInterfaceImporter : public LuaContextWatcher
{
 public:
  /** Map for interfaces. */
  typedef fawkes::LockMap<std::string, fawkes::Interface *>  InterfaceMap;


  LuaInterfaceImporter(LuaContext *__context, BlackBoard *blackboard,
		       Configuration *config, Logger *logger);
  ~LuaInterfaceImporter();

  void open_reading_interfaces(std::string &prefix);
  void open_writing_interfaces(std::string &prefix);

  void add_interface(std::string varname, Interface *interface);

  void close_reading_interfaces();
  void close_writing_interfaces();

  LuaInterfaceImporter::InterfaceMap & writing_interfaces();
  LuaInterfaceImporter::InterfaceMap & reading_interfaces();

  void push_interfaces();

  void read();
  void write();

  void lua_restarted(LuaContext *context);

 private:
  void open_interfaces(std::string &prefix, InterfaceMap &imap, bool write);
  void push_interfaces(LuaContext *context);
  void push_interfaces_varname(LuaContext *context, InterfaceMap &imap);
  void push_interfaces_uid(LuaContext *context, InterfaceMap &imap);

 private:
  LuaContext    *__context;
  BlackBoard    *__blackboard;
  Configuration *__config;
  Logger        *__logger;

  InterfaceMap   __reading_ifs;
  InterfaceMap   __writing_ifs;

  InterfaceMap   __ext_rifs;
  InterfaceMap   __ext_wifs;

  bool           __interfaces_pushed;
};

} // end of namespace fawkes

#endif
