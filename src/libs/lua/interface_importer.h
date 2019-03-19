
/***************************************************************************
 *  interfaceimporter.h - Fawkes Lua Interface Importer
 *
 *  Created: Thu Jan 01 14:28:47 2009
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

#ifndef _LUA_INTERFACEIMPORTER_H_
#define _LUA_INTERFACEIMPORTER_H_

#include <blackboard/interface_observer.h>
#include <core/utils/lock_map.h>
#include <lua/context_watcher.h>

#include <list>
#include <string>

namespace fawkes {

class BlackBoard;
class Configuration;
class Interface;
class Logger;
class LuaContext;

class LuaInterfaceImporter : public LuaContextWatcher
{
	class InterfaceObserver : public BlackBoardInterfaceObserver
	{
	public:
		InterfaceObserver(LuaInterfaceImporter *lii,
		                  std::string           varname,
		                  const char *          type,
		                  const char *          id_pattern);

		virtual void bb_interface_created(const char *type, const char *id) throw();

	private:
		LuaInterfaceImporter *lii_;
		std::string           varname_;
	};

	typedef fawkes::LockMap<std::string, InterfaceObserver *> ObserverMap;

public:
	/** Map of varname to interface instance. */
	typedef fawkes::LockMap<std::string, fawkes::Interface *> InterfaceMap;
	/** Map of varname to list of interfaces */
	typedef fawkes::LockMap<std::string, std::list<fawkes::Interface *>> InterfaceListMap;

	LuaInterfaceImporter(LuaContext *   context_,
	                     BlackBoard *   blackboard,
	                     Configuration *config,
	                     Logger *       logger);
	~LuaInterfaceImporter();

	void open_reading_interfaces(std::string &prefix);
	void open_writing_interfaces(std::string &prefix);

	void add_interface(std::string varname, Interface *interface);

	void close_reading_interfaces();
	void close_writing_interfaces();

	LuaInterfaceImporter::InterfaceMap &writing_interfaces();
	LuaInterfaceImporter::InterfaceMap &reading_interfaces();

	void push_interfaces();

	void read_to_buffer();
	void read_from_buffer();

	void read();
	void write();

	void lua_restarted(LuaContext *context);

private:
	void open_interfaces(std::string &prefix, InterfaceMap &imap, bool write);
	void push_interfaces(LuaContext *context);
	void push_interfaces_varname(LuaContext *context, InterfaceMap &imap);
	void push_interfaces_uid(LuaContext *context, InterfaceMap &imap);
	void push_multi_interfaces_varname(LuaContext *context, InterfaceListMap &imap);

	void add_observed_interface(std::string varname, const char *type, const char *id);

private:
	LuaContext *   context_;
	BlackBoard *   blackboard_;
	Configuration *config_;
	Logger *       logger_;

	bool two_stage_;

	InterfaceMap     reading_ifs_;
	InterfaceListMap reading_multi_ifs_;
	InterfaceMap     writing_ifs_;
	ObserverMap      observers_;

	InterfaceMap ext_rifs_;
	InterfaceMap ext_wifs_;

	bool interfaces_pushed_;
};

} // end of namespace fawkes

#endif
