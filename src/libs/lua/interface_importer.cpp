
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

#include <blackboard/blackboard.h>
#include <config/config.h>
#include <interface/interface.h>
#include <logging/logger.h>
#include <lua/context.h>
#include <lua/interface_importer.h>

#include <cstring>

namespace fawkes {

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
LuaInterfaceImporter::LuaInterfaceImporter(LuaContext *   context,
                                           BlackBoard *   blackboard,
                                           Configuration *config,
                                           Logger *       logger)
{
	context_    = context;
	blackboard_ = blackboard;
	config_     = config;
	logger_     = logger;
	two_stage_  = false;
	context_->add_watcher(this);

	interfaces_pushed_ = false;
}

/** Destructor. */
LuaInterfaceImporter::~LuaInterfaceImporter()
{
	context_->remove_watcher(this);
	close_writing_interfaces();
	close_reading_interfaces();
	ext_rifs_.clear();
	ext_wifs_.clear();
}

/** Open interfaces (internal).
 * @param prefix configuration prefix for the interface list
 * @param imap interface map to fill with interfaces
 * @param write if true interfaces are opened for writing, false to open for reading
 */
void
LuaInterfaceImporter::open_interfaces(std::string &prefix, InterfaceMap &imap, bool write)
{
	if (!config_)
		throw NullPointerException("Config has not been set");

	Configuration::ValueIterator *vi = config_->search(prefix.c_str());
	while (vi->next()) {
		if (strcmp(vi->type(), "string") != 0) {
			TypeMismatchException e("Only values of type string may occur in %s, "
			                        "but found value of type %s",
			                        prefix.c_str(),
			                        vi->type());
			delete vi;
			throw e;
		}
		std::string uid = vi->get_string();

		if (uid.find("::") == std::string::npos) {
			delete vi;
			throw Exception("Interface UID '%s' at %s is not valid, missing double colon",
			                uid.c_str(),
			                vi->path());
		}
		std::string varname = std::string(vi->path()).substr(prefix.length());
		std::string iftype  = uid.substr(0, uid.find("::"));
		std::string ifname  = uid.substr(uid.find("::") + 2);

		if (reading_ifs_.find(varname) != reading_ifs_.end()) {
			delete vi;
			throw Exception("Reading interface with varname %s already opened", varname.c_str());
		}
		if (reading_multi_ifs_.find(varname) != reading_multi_ifs_.end()) {
			delete vi;
			throw Exception("Reading multi interface with varname %s already opened", varname.c_str());
		}
		if (writing_ifs_.find(varname) != writing_ifs_.end()) {
			delete vi;
			throw Exception("Writing interface with varname %s already opened", varname.c_str());
		}

		if (ifname.find_first_of("*?[") == std::string::npos) {
			logger_->log_info("LuaInterfaceImporter",
			                  "Adding %s interface %s::%s with name %s",
			                  write ? "writing" : "reading",
			                  iftype.c_str(),
			                  ifname.c_str(),
			                  varname.c_str());
			try {
				Interface *iface;
				if (write) {
					iface = blackboard_->open_for_writing(iftype.c_str(), ifname.c_str());
				} else {
					iface = blackboard_->open_for_reading(iftype.c_str(), ifname.c_str());
				}
				if (two_stage_) {
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
				                "only be opened for reading",
				                vi->path(),
				                uid.c_str());
			}
			logger_->log_info("LuaInterfaceImporter",
			                  "Adding multiple %s interfaces %s::%s with in table %s",
			                  write ? "writing" : "reading",
			                  iftype.c_str(),
			                  ifname.c_str(),
			                  varname.c_str());

			std::list<Interface *> interfaces =
			  blackboard_->open_multiple_for_reading(iftype.c_str(), ifname.c_str());
			reading_multi_ifs_[varname] = interfaces;
			InterfaceObserver *observer =
			  new InterfaceObserver(this, varname, iftype.c_str(), ifname.c_str());
			observers_[varname] = observer;
			blackboard_->register_observer(observer);
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
	open_interfaces(prefix, reading_ifs_, /* write */ false);
}

/** Open interfaces for writing.
 * @param prefix configuration prefix for the interface list
 */
void
LuaInterfaceImporter::open_writing_interfaces(std::string &prefix)
{
	open_interfaces(prefix, writing_ifs_, /* write */ true);
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
	if (interface->is_writer()) {
		ext_wifs_[varname] = interface;
	} else {
		ext_rifs_[varname] = interface;
	}
}

void
LuaInterfaceImporter::add_observed_interface(std::string varname, const char *type, const char *id)
{
	try {
		if (reading_multi_ifs_.find(varname) == reading_multi_ifs_.end()) {
			throw Exception("Notified about unknown interface varname %s", varname.c_str());
		}
		Interface *iface = blackboard_->open_for_reading(type, id);
		context_->add_package((std::string("interfaces.") + iface->type()).c_str());
		reading_multi_ifs_[varname].push_back(iface);
		context_->get_global("interfaces");                         // it
		context_->get_field(-1, "reading");                         // it rt
		context_->get_field(-1, varname.c_str());                   // it rt vt
		context_->push_usertype(iface, iface->type(), "fawkes");    // it rt vt iface
		context_->raw_seti(-2, reading_multi_ifs_[varname].size()); // it rt vt
		context_->push_usertype(iface, iface->type(), "fawkes");    // it rt vt iface
		context_->set_field(iface->uid(), -2);                      // it rt vt
		context_->pop(3);                                           // ---
	} catch (Exception &e) {
		logger_->log_warn("LuaInterfaceImporter",
		                  "Failed to add observed interface "
		                  "%s:%s, exception follows",
		                  type,
		                  id);
		logger_->log_warn("LuaInterfaceImporter", e);
	}
}

/** Close interfaces for reading. */
void
LuaInterfaceImporter::close_reading_interfaces()
{
	for (InterfaceMap::iterator i = reading_ifs_.begin(); i != reading_ifs_.end(); ++i) {
		blackboard_->close(i->second);
	}
	reading_ifs_.clear();

	for (ObserverMap::iterator o = observers_.begin(); o != observers_.end(); ++o) {
		blackboard_->unregister_observer(o->second);
		delete o->second;
	}
	observers_.clear();

	for (InterfaceListMap::iterator i = reading_multi_ifs_.begin(); i != reading_multi_ifs_.end();
	     ++i) {
		for (std::list<Interface *>::iterator j = i->second.begin(); j != i->second.end(); ++j) {
			blackboard_->close(*j);
		}
	}
	reading_multi_ifs_.clear();
}

/** Close interfaces for writing. */
void
LuaInterfaceImporter::close_writing_interfaces()
{
	for (InterfaceMap::iterator i = writing_ifs_.begin(); i != writing_ifs_.end(); ++i) {
		blackboard_->close(i->second);
	}
	writing_ifs_.clear();
}

/** Get interface map of reading interfaces.
 * @return interface map of reading interfaces
 */
LuaInterfaceImporter::InterfaceMap &
LuaInterfaceImporter::reading_interfaces()
{
	return reading_ifs_;
}

/** Get interface map of writing interfaces.
 * @return interface map of writing interfaces
 */
LuaInterfaceImporter::InterfaceMap &
LuaInterfaceImporter::writing_interfaces()
{
	return writing_ifs_;
}

/** Read from all reading interfaces. */
void
LuaInterfaceImporter::read()
{
	for (InterfaceMap::iterator i = reading_ifs_.begin(); i != reading_ifs_.end(); ++i) {
		i->second->read();
	}
}

/** Read from all reading interfaces into a buffer.
 */
void
LuaInterfaceImporter::read_to_buffer()
{
	InterfaceMap::iterator i;
	if (!two_stage_) {
		for (i = reading_ifs_.begin(); i != reading_ifs_.end(); ++i) {
			i->second->resize_buffers(1);
		}
		two_stage_ = true;
	}
	for (i = reading_ifs_.begin(); i != reading_ifs_.end(); ++i) {
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
	if (!two_stage_) {
		throw Exception("LuaInterfaceImporter: trying to read buffer witout "
		                "previous read_to_buffer()");
	}
	InterfaceMap::iterator i;
	for (i = reading_ifs_.begin(); i != reading_ifs_.end(); ++i) {
		i->second->read_from_buffer(0);
	}
}

/** Write all writing interfaces. */
void
LuaInterfaceImporter::write()
{
	for (InterfaceMap::iterator i = writing_ifs_.begin(); i != writing_ifs_.end(); ++i) {
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
	context->create_table(0, 4); // it

	context->create_table(0, reading_ifs_.size() + ext_rifs_.size()); // it rt
	push_interfaces_varname(context, reading_ifs_);                   // it rt
	push_interfaces_varname(context, ext_rifs_);                      // it rt
	push_multi_interfaces_varname(context, reading_multi_ifs_);       // it rt
	context->set_field("reading");                                    // it

	context->create_table(0, reading_ifs_.size() + ext_rifs_.size()); // it rtu
	push_interfaces_uid(context, reading_ifs_);                       // it rtu
	push_interfaces_uid(context, ext_rifs_);                          // it rtu
	context->set_field("reading_by_uid");                             // it

	context->create_table(0, writing_ifs_.size() + ext_wifs_.size()); // it wt
	push_interfaces_varname(context, writing_ifs_);                   // it wt
	push_interfaces_varname(context, ext_wifs_);                      // it wt
	context->set_field("writing");                                    // it

	context->create_table(0, writing_ifs_.size()); // it wtu
	push_interfaces_uid(context, writing_ifs_);    // it wtu
	push_interfaces_uid(context, ext_wifs_);       // it wtu
	context->set_field("writing_by_uid");          // it

	context->set_global("interfaces"); // ---
}

/** Push interfaces to Lua environment.
 * The interfaces are pushed to the interfaces table described in the class
 * documentation. Note that you need to do this only once. The table is
 * automatically re-pushed on a Lua restart event.
 */
void
LuaInterfaceImporter::push_interfaces()
{
	interfaces_pushed_ = true;
	push_interfaces(context_);
}

void
LuaInterfaceImporter::lua_restarted(LuaContext *context)
{
	try {
		if (interfaces_pushed_) {
			push_interfaces(context);
		}
	} catch (Exception &e) {
		logger_->log_warn("LuaInterfaceImporter", "Failed to re-push interfacs, exception follows");
		logger_->log_warn("LuaInterfaceImporter", e);
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
                                                           std::string           varname,
                                                           const char *          type,
                                                           const char *          id_pattern)
{
	lii_     = lii;
	varname_ = varname;

	bbio_add_observed_create(type, id_pattern);
}

void
LuaInterfaceImporter::InterfaceObserver::bb_interface_created(const char *type,
                                                              const char *id) throw()
{
	lii_->add_observed_interface(varname_, type, id);
}

} // end of namespace fawkes
