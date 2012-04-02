
/***************************************************************************
 *  interfaceimporter.cpp - Fawkes Lua Interface Importer
 *
 *  Created: Thu Jan 01 14:32:11 2009
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

#include <lua/interface_importer.h>
#include <lua/context.h>

#include <config/config.h>
#include <interface/interface.h>
#include <blackboard/blackboard.h>
#include <logging/logger.h>

#include <cstring>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LuaInterfaceImporter <lua/interface_importer.h>
 * Lua interface importer.
 * The Lua interface importer reads a list from the configuration for a given
 * prefix and exports them to the Lua environment. The configuration entries have
 * the form "/this/is/the/prefix/variablename" -> Interface UID. The interfaces
 * are exported as a table assigned to the global variable named "interfaces".
 * This table has four entries, reading and writing to tables with variablename
 * to interface mappings and reading_by_uid and writing_by_uid with mappings from
 * the interface UID to the interface.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param context Lua context
 * @param blackboard BlackBoard
 * @param config configuration
 * @param logger Logger
 */
LuaInterfaceImporter::LuaInterfaceImporter(LuaContext *context,
					   BlackBoard *blackboard,
					   Configuration *config,
					   Logger *logger)
{
  __context = context;
  __blackboard = blackboard;
  __config = config;
  __logger = logger;
  __two_stage = false;
  __context->add_watcher(this);

  __interfaces_pushed = false;
}


/** Destructor. */
LuaInterfaceImporter::~LuaInterfaceImporter()
{
  __context->remove_watcher(this);
  close_writing_interfaces();
  close_reading_interfaces();
  __ext_rifs.clear();
  __ext_wifs.clear();
}


/** Open interfaces (internal).
 * @param prefix configuration prefix for the interface list
 * @param imap interface map to fill with interfaces
 * @param write if true interfaces are opened for writing, false to open for reading
 */
void
LuaInterfaceImporter::open_interfaces(std::string &prefix, InterfaceMap &imap, bool write)
{
  if (! __config) throw NullPointerException("Config has not been set");

  Configuration::ValueIterator *vi = __config->search(prefix.c_str());
  while (vi->next()) {
    if (strcmp(vi->type(), "string") != 0) {
      TypeMismatchException e("Only values of type string may occur in %s, "
			      "but found value of type %s",
			      prefix.c_str(), vi->type());
      delete vi;
      throw e;
    }
    std::string uid = vi->get_string();

    if (uid.find("::") == std::string::npos) {
      delete vi;
      throw Exception("Interface UID '%s' at %s is not valid, missing double colon",
		      uid.c_str(), vi->path());
    }
    std::string varname = std::string(vi->path()).substr(prefix.length());
    std::string iftype = uid.substr(0, uid.find("::"));
    std::string ifname = uid.substr(uid.find("::") + 2);

    if ( __reading_ifs.find(varname) != __reading_ifs.end() ) {
      delete vi;
      throw Exception("Reading interface with varname %s already opened", varname.c_str());
    }
    if ( __reading_multi_ifs.find(varname) != __reading_multi_ifs.end() ) {
      delete vi;
      throw Exception("Reading multi interface with varname %s already opened", varname.c_str());
    }
    if ( __writing_ifs.find(varname) != __writing_ifs.end() ) {
      delete vi;
      throw Exception("Writing interface with varname %s already opened", varname.c_str());
    }


    if (ifname.find_first_of("*?[") == std::string::npos) {
      __logger->log_info("LuaInterfaceImporter", "Adding %s interface %s::%s with name %s",
			 write ? "writing" : "reading",
			 iftype.c_str(), ifname.c_str(), varname.c_str());
      try {
	Interface *iface;
	if (write) {
	  iface = __blackboard->open_for_writing(iftype.c_str(), ifname.c_str());
	} else {
	  iface = __blackboard->open_for_reading(iftype.c_str(), ifname.c_str());
	}
	if (__two_stage) {
	  iface->resize_buffers(1);
	}
	imap[varname] = iface;
      } catch (Exception &e) {
	delete vi;
	throw;
      }
    } else {
      if (write) {
	delete vi;
	throw Exception("Illegal config entry %s=%s, multiple interfaces can "
			"only be opened for reading", vi->path(), uid.c_str());
      }
      __logger->log_info("LuaInterfaceImporter", "Adding multiple %s interfaces %s::%s with in table %s",
			 write ? "writing" : "reading",
			 iftype.c_str(), ifname.c_str(), varname.c_str());

      std::list<Interface *> interfaces = __blackboard->open_multiple_for_reading(iftype.c_str(), ifname.c_str());
      __reading_multi_ifs[varname] = interfaces;
      InterfaceObserver *observer =
        new InterfaceObserver(this, varname, iftype.c_str(), ifname.c_str());
      __observers[varname] = observer;
      __blackboard->register_observer(observer);
    }
  }
  delete vi;
}


/** Open interfaces for reading.
 * @param prefix configuration prefix for the interface list
 */
void
LuaInterfaceImporter::open_reading_interfaces(std::string &prefix)
{
  open_interfaces(prefix, __reading_ifs, /* write */ false);
}

/** Open interfaces for writing.
 * @param prefix configuration prefix for the interface list
 */
void
LuaInterfaceImporter::open_writing_interfaces(std::string &prefix)
{
  open_interfaces(prefix, __writing_ifs, /* write */ true);
}


/** Add a single interface to be pushed to the context.
 * The given interface is pushed with the given variable name to the context,
 * on explicit push_interfaces() and on the next LuaContext restart. However, the
 * interface is not owned by the importer and thus neither is the interface read
 * during read() nor is it written during write(). It is also not automatically
 * closed in the destructor.
 * @param varname the variable name of the interface
 * @param interface the interface to push
 */
void
LuaInterfaceImporter::add_interface(std::string varname, Interface *interface)
{
  if ( interface->is_writer() ) {
    __ext_wifs[varname] = interface;
  } else {
    __ext_rifs[varname] = interface;
  }
}


void
LuaInterfaceImporter::add_observed_interface(std::string varname,
					     const char *type, const char *id)
{
  try {
    if (__reading_multi_ifs.find(varname) == __reading_multi_ifs.end() ) {
      throw Exception("Notified about unknown interface varname %s", varname.c_str());
    }
    Interface *iface = __blackboard->open_for_reading(type, id);
    __context->add_package((std::string("interfaces.") + iface->type()).c_str());
    __reading_multi_ifs[varname].push_back(iface);
    __context->get_global("interfaces");			// it
    __context->get_field(-1, "reading");			// it rt
    __context->get_field(-1, varname.c_str());			// it rt vt
    __context->push_usertype(iface, iface->type(), "fawkes");	// it rt vt iface
    __context->raw_seti(-2, __reading_multi_ifs[varname].size()); // it rt vt
    __context->push_usertype(iface, iface->type(), "fawkes");	// it rt vt iface
    __context->set_field(iface->uid(), -2);			// it rt vt
    __context->pop(3);						// ---
 } catch (Exception &e) {
    __logger->log_warn("LuaInterfaceImporter", "Failed to add observed interface "
		       "%s:%s, exception follows", type, id);
    __logger->log_warn("LuaInterfaceImporter", e);    
  }
}


/** Close interfaces for reading. */
void
LuaInterfaceImporter::close_reading_interfaces()
{
  for (InterfaceMap::iterator i = __reading_ifs.begin(); i != __reading_ifs.end(); ++i) {
    __blackboard->close(i->second);
  }
  __reading_ifs.clear();

  for (ObserverMap::iterator o = __observers.begin(); o != __observers.end(); ++o) {
    __blackboard->unregister_observer(o->second);
    delete o->second;
  }
  __observers.clear();

  for (InterfaceListMap::iterator i = __reading_multi_ifs.begin(); i != __reading_multi_ifs.end(); ++i) {
    for (std::list<Interface *>::iterator j = i->second.begin(); j != i->second.end(); ++j) {
      __blackboard->close(*j);
    }
  }
  __reading_multi_ifs.clear();
}


/** Close interfaces for writing. */
void
LuaInterfaceImporter::close_writing_interfaces()
{
  for (InterfaceMap::iterator i = __writing_ifs.begin(); i != __writing_ifs.end(); ++i) {
    __blackboard->close(i->second);
  }
  __writing_ifs.clear();
}

/** Get interface map of reading interfaces.
 * @return interface map of reading interfaces
 */
LuaInterfaceImporter::InterfaceMap &
LuaInterfaceImporter::reading_interfaces()
{
  return __reading_ifs;
}


/** Get interface map of writing interfaces.
 * @return interface map of writing interfaces
 */
LuaInterfaceImporter::InterfaceMap &
LuaInterfaceImporter::writing_interfaces()
{
  return __writing_ifs;
}


/** Read from all reading interfaces. */
void
LuaInterfaceImporter::read()
{
  for (InterfaceMap::iterator i = __reading_ifs.begin(); i != __reading_ifs.end(); ++i) {
    i->second->read();
  }
}


/** Read from all reading interfaces into a buffer.
 */
void
LuaInterfaceImporter::read_to_buffer()
{
  InterfaceMap::iterator i;
  if (! __two_stage) {
    for (i = __reading_ifs.begin(); i != __reading_ifs.end(); ++i) {
      i->second->resize_buffers(1);
    }
    __two_stage = true;
  }
  for (i = __reading_ifs.begin(); i != __reading_ifs.end(); ++i) {
    i->second->copy_shared_to_buffer(0);
  }
}

/** Update interfaces from internal buffers.
 * @exception Exception thrown if read_to_buffer() was not called
 * before.
 */
void
LuaInterfaceImporter::read_from_buffer()
{
  if (! __two_stage) {
    throw Exception("LuaInterfaceImporter: trying to read buffer witout "
		    "previous read_to_buffer()");
  }
  InterfaceMap::iterator i;
  for (i = __reading_ifs.begin(); i != __reading_ifs.end(); ++i) {
    i->second->read_from_buffer(0);
  }
}

/** Write all writing interfaces. */
void
LuaInterfaceImporter::write()
{
  for (InterfaceMap::iterator i = __writing_ifs.begin(); i != __writing_ifs.end(); ++i) {
    try {
      i->second->write();
    } catch (Exception &e) {
      e.append("Failed to write interface %s, ignoring.", i->second->uid());
      e.print_trace();
    }
  }
}

void
LuaInterfaceImporter::push_interfaces_varname(LuaContext *context, InterfaceMap &imap)
{
  InterfaceMap::iterator imi;
  for (imi = imap.begin(); imi != imap.end(); ++imi) {
    context->add_package((std::string("interfaces.") + imi->second->type()).c_str());
    context->push_usertype(imi->second, imi->second->type(), "fawkes");
    context->set_field(imi->first.c_str());
  }
}

void
LuaInterfaceImporter::push_multi_interfaces_varname(LuaContext *context, InterfaceListMap &imap)
{
  InterfaceListMap::iterator imi;
  for (imi = imap.begin(); imi != imap.end(); ++imi) {
    context->create_table(0, imi->second.size());
    int idx = 0;
    for (std::list<Interface *>::iterator i = imi->second.begin(); i != imi->second.end(); ++i) {
      context->add_package((std::string("interfaces.") + (*i)->type()).c_str());
      context->push_usertype(*i, (*i)->type(), "fawkes");
      context->raw_seti(-2, ++idx);
      context->push_usertype(*i, (*i)->type(), "fawkes");
      context->set_field((*i)->uid(), -2);
    }
    context->set_field(imi->first.c_str());
  }
}

void
LuaInterfaceImporter::push_interfaces_uid(LuaContext *context, InterfaceMap &imap)
{
  InterfaceMap::iterator imi;
  for (imi = imap.begin(); imi != imap.end(); ++imi) {
    context->add_package((std::string("interfaces.") + imi->second->type()).c_str());
    context->push_usertype(imi->second, imi->second->type(), "fawkes");
    context->set_field(imi->second->uid());
  }
}

void
LuaInterfaceImporter::push_interfaces(LuaContext *context)
{

  // it: interface table, rt: reading table, wt: writing table, rtu: rt by uid, wtu: wt by uid
  context->create_table(0, 4);				// it

  context->create_table(0, __reading_ifs.size() + __ext_rifs.size());	// it rt
  push_interfaces_varname(context, __reading_ifs);	// it rt
  push_interfaces_varname(context, __ext_rifs);		// it rt
  push_multi_interfaces_varname(context, __reading_multi_ifs);	// it rt
  context->set_field("reading");			// it

  context->create_table(0, __reading_ifs.size() + __ext_rifs.size());	// it rtu
  push_interfaces_uid(context, __reading_ifs);		// it rtu
  push_interfaces_uid(context, __ext_rifs);		// it rtu
  context->set_field("reading_by_uid");			// it

  context->create_table(0, __writing_ifs.size() + __ext_wifs.size());	// it wt
  push_interfaces_varname(context, __writing_ifs);		// it wt
  push_interfaces_varname(context, __ext_wifs);		// it wt
  context->set_field("writing");			// it

  context->create_table(0, __writing_ifs.size());	// it wtu
  push_interfaces_uid(context, __writing_ifs);		// it wtu
  push_interfaces_uid(context, __ext_wifs);		// it wtu
  context->set_field("writing_by_uid");			// it

  context->set_global("interfaces");			// ---
}

/** Push interfaces to Lua environment.
 * The interfaces are pushed to the interfaces table described in the class
 * documentation. Note that you need to do this only once. The table is
 * automatically re-pushed on a Lua restart event.
 */
void
LuaInterfaceImporter::push_interfaces()
{
  __interfaces_pushed = true;
  push_interfaces(__context);
}


void
LuaInterfaceImporter::lua_restarted(LuaContext *context)
{
  try {
    if ( __interfaces_pushed ) {
      push_interfaces(context);
    }
  } catch (Exception &e) {
    __logger->log_warn("LuaInterfaceImporter", "Failed to re-push interfacs, exception follows");
    __logger->log_warn("LuaInterfaceImporter", e);
    throw;
  }
}


/** Constructor.
 * @param lii LuaInterfaceImporter instance this observer is assigned to
 * @param varname variable name
 * @param type type of interface
 * @param id_pattern ID pattern to observe
 */
LuaInterfaceImporter::InterfaceObserver::InterfaceObserver(LuaInterfaceImporter *lii,
							   std::string varname,
							   const char *type, const char *id_pattern)
{
  __lii = lii;
  __varname = varname;

  bbio_add_observed_create(type, id_pattern);
}


void
LuaInterfaceImporter::InterfaceObserver::bb_interface_created(const char *type, const char *id) throw()
{
  __lii->add_observed_interface(__varname, type, id);
}

} // end of namespace fawkes
