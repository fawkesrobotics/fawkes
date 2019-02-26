
/***************************************************************************
 *  blackboard_adapter.cpp - PLEXIL adapter for Fawkes' blackboard
 *
 *  Created: Sun Feb 17 12:52:45 2019 +0100
 *  Copyright  2006-2019  Tim Niemueller [www.niemueller.de]
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

#include "blackboard_adapter.h"
#include "utils.h"

#include <utils/misc/string_conversions.h>
#include <utils/time/time.h>

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <State.hh>
#include <Command.hh>
#include <Error.hh>
#include <StateCacheEntry.hh>
#include <CachedValue.hh>

using namespace fawkes;

/** @class BlackboardPlexilAdapter "blackboard_adapter.h"
 * Plexil adapter to provide access to Fawkes blackboard
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
BlackboardPlexilAdapter::BlackboardPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface)
: InterfaceAdapter(execInterface),
  BlackBoardInterfaceListener("PlexilBB")
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
BlackboardPlexilAdapter::BlackboardPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
                                                 pugi::xml_node const xml)
: InterfaceAdapter(execInterface, xml),
  BlackBoardInterfaceListener("PlexilBB")
{
}

/** Destructor. */
BlackboardPlexilAdapter::~BlackboardPlexilAdapter()
{
}


/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
BlackboardPlexilAdapter::initialize()
{
	logger_  = reinterpret_cast<fawkes::Logger *>(m_execInterface.getProperty("::Fawkes::Logger"));
	blackboard_ =
	  reinterpret_cast<fawkes::BlackBoard *>(m_execInterface.getProperty("::Fawkes::BlackBoard"));

	namespace p = std::placeholders;
	commands_ = {
	  {"BB_open_for_reading",  std::bind(&BlackboardPlexilAdapter::bb_open_for_reading, this, p::_1)},
	  {"BB_close",  std::bind(&BlackboardPlexilAdapter::bb_close, this, p::_1)},
	  {"BB_read",  std::bind(&BlackboardPlexilAdapter::bb_read, this, p::_1)},
	  {"BB_read_all",  std::bind(&BlackboardPlexilAdapter::bb_read_all, this, p::_1)},
	  {"BB_print",  std::bind(&BlackboardPlexilAdapter::bb_print, this, p::_1)}
	};

	std::vector<std::string> lookups = { "BB_changed", "BB_int", "BB_real", "BB_bool",
	                                     "BB_string", "BB_field_length",
	                                     "BB_int_at", "BB_real_at", "BB_bool_at" };

	for (const auto &c: commands_) {
		PLEXIL::g_configuration->registerCommandInterface(c.first, this);
	}

	for (const auto &l: lookups) {
		PLEXIL::g_configuration->registerLookupInterface(l, this);
	}

	blackboard_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);

	return true;
}


/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
BlackboardPlexilAdapter::start()
{
	return true;
}


/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
BlackboardPlexilAdapter::stop()
{
	return true;
}


/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
BlackboardPlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
BlackboardPlexilAdapter::shutdown()
{
	blackboard_->unregister_listener(this);

	for (const auto& if_entry: ifs_read_) {
		debugMsg("BlackboardAdapter:close",
		         "Closing " << if_entry.second->type() << "::" << if_entry.second->id());
		blackboard_->close(if_entry.second);
	}
	ifs_read_.clear();
	return true;
}

/** Immediate lookup of value.
 * @param state state variable to lookup
 * @param cache_entry cache entry for retrieved value
 */
void
BlackboardPlexilAdapter::lookupNow(PLEXIL::State const &state, PLEXIL::StateCacheEntry &cache_entry)
{
	std::vector<PLEXIL::Value> const &params = state.parameters();
	if (state.name() == "BB_changed") {
		if (! verify_args(params, "BlackboardAdapter:BB_changed", {{"uid", PLEXIL::STRING_TYPE}})) {
			cache_entry.setUnknown();
			return;
		}
		std::string uid;
		params[0].getValue(uid);

		std::unique_lock<std::mutex> lock(ifs_read_mutex_);
		if (ifs_read_.find(uid) == ifs_read_.end()) {
			logger_->log_warn("PlexilBB", "BB_changed: unknown interface %s, forgot to open?",
			                  uid.c_str());
			cache_entry.setUnknown();
			return;
		}
		cache_entry.update(ifs_read_[uid]->changed());
	} else if (state.name() == "BB_int" || state.name() == "BB_int_at" ||
	           state.name() == "BB_real" || state.name() == "BB_real_at" ||
	           state.name() == "BB_bool" || state.name() == "BB_bool_at" ||
	           state.name() == "BB_string" || state.name() == "BB_field_length")
	{
		bool is_indexed_version = false;
		if (state.name() == "BB_int" || state.name() == "BB_real" ||
		    state.name() == "BB_bool" || state.name() == "BB_string" ||
		    state.name() == "BB_field_length")
		{
			if (! verify_args(params, "BlackboardAdapter:"+state.name(),
			                  {{"uid", PLEXIL::STRING_TYPE},
			                   {"field", PLEXIL::STRING_TYPE}}))
			{
				cache_entry.setUnknown();
				return;
			}
		} else {
			is_indexed_version = true;
			if (! verify_args(params, "BlackboardAdapter:"+state.name(),
			                  {{"uid", PLEXIL::STRING_TYPE},
			                   {"field", PLEXIL::STRING_TYPE},
			                   {"index", PLEXIL::INTEGER_TYPE}}))
			{
				cache_entry.setUnknown();
				return;
			}
		}
		std::string uid;
		std::string field;
		int index = 0;
		params[0].getValue(uid);
		params[1].getValue(field);

		if (is_indexed_version) {
			params[2].getValue(index);
		}

		std::unique_lock<std::mutex> lock(ifs_read_mutex_);
		if (ifs_read_.find(uid) == ifs_read_.end()) {
			logger_->log_warn("PlexilBB", "%s: unknown interface %s, forgot to open?",
			                  state.name().c_str(), uid.c_str());
			cache_entry.setUnknown();
			return;
		}
		bool found = false;
		if (state.name().compare(0, 6, "BB_int") == 0) {
			for (auto f = ifs_read_[uid]->fields(); f != ifs_read_[uid]->fields_end(); ++f) {
				if (field == f.get_name()) {
					found = true;
					bool valid = true;
					PLEXIL::Integer value;
					switch (f.get_type()) {
					case IFT_INT8:   value = f.get_int8(index);    break;
					case IFT_UINT8:  value = f.get_uint8(index);   break;
					case IFT_INT16:  value = f.get_int16(index);   break;
					case IFT_UINT16: value = f.get_uint16(index);  break;
					case IFT_INT32:  value = f.get_int32(index);   break;
					case IFT_UINT32: value = f.get_uint32(index);  break;
					case IFT_INT64:  value = f.get_int64(index);   break;
					case IFT_UINT64: value = f.get_uint64(index);  break;
					default: valid = false; break;
					}
					if (valid) {
						cache_entry.update(value);
					} else {
						logger_->log_warn("PlexilBB", "BB_int: field '%s' of %s of type %s",
						                  field.c_str(), uid.c_str(), f.get_typename());
						cache_entry.setUnknown();
					}
				}
				break;
			}
		} else if (state.name().compare(0, 7, "BB_real") == 0) {
			for (auto f = ifs_read_[uid]->fields(); f != ifs_read_[uid]->fields_end(); ++f) {
				if (field == f.get_name()) {
					found = true;
					bool valid = true;
					PLEXIL::Real value;
					switch (f.get_type()) {
					case IFT_FLOAT:  value = f.get_float(index);   break;
					case IFT_DOUBLE: value = f.get_double(index);  break;
					case IFT_INT8:   value = f.get_int8(index);    break;
					case IFT_UINT8:  value = f.get_uint8(index);   break;
					case IFT_INT16:  value = f.get_int16(index);   break;
					case IFT_UINT16: value = f.get_uint16(index);  break;
					case IFT_INT32:  value = f.get_int32(index);   break;
					case IFT_UINT32: value = f.get_uint32(index);  break;
					case IFT_INT64:  value = f.get_int64(index);   break;
					case IFT_UINT64: value = f.get_uint64(index);  break;
					default: valid = false; break;
					}
					if (valid) {
						cache_entry.update(value);
					} else {
						logger_->log_warn("PlexilBB", "BB_int: field %s of %s of type %s",
						                  field.c_str(), uid.c_str(), f.get_typename());
						cache_entry.setUnknown();
					}
				}
				break;
			}
		} else if (state.name().compare(0, 7, "BB_bool") == 0) {
			for (auto f = ifs_read_[uid]->fields(); f != ifs_read_[uid]->fields_end(); ++f) {
				if (field == f.get_name()) {
					found = true;
					PLEXIL::Boolean value;
					if (f.get_type() == IFT_BOOL) {
						value = f.get_bool(index);
						cache_entry.update(value);
					} else {
						logger_->log_warn("PlexilBB", "BB_int: field %s of %s of type %s",
						                  field.c_str(), uid.c_str(), f.get_typename());
						cache_entry.setUnknown();
					}
				}
				break;
			}
		} else if (state.name() == "BB_string") {
			for (auto f = ifs_read_[uid]->fields(); f != ifs_read_[uid]->fields_end(); ++f) {
				if (field == f.get_name()) {
					found = true;
					PLEXIL::String value;
					value = f.get_value_string();
					cache_entry.update(value);
				}
				break;
			}
		} else if (state.name() == "BB_field_length") {
			for (auto f = ifs_read_[uid]->fields(); f != ifs_read_[uid]->fields_end(); ++f) {
				if (field == f.get_name()) {
					found = true;
					PLEXIL::Integer value;
					value = f.get_length();
					cache_entry.update(value);
				}
				break;
			}
		}
		if (! found) {
			logger_->log_warn("PlexilBB", "%s: unknown field '%s' for interface %s",
			                  state.name().c_str(), field.c_str(), uid.c_str());
			cache_entry.setUnknown();
			return;
		}

	} else {
		logger_->log_warn("PlexilBB", "unknown lookup '%s'",
		                  state.name().c_str());
		cache_entry.setUnknown();
		return;
	}
}


/** Subscribe to updates for given state.
 * @param state state variable to subscribe for
 */
void
BlackboardPlexilAdapter::subscribe(const PLEXIL::State& state)
{
	std::vector<PLEXIL::Value> const &params = state.parameters();
	if (params.size() == 0 || params[0].valueType() != PLEXIL::STRING_TYPE) {
		logger_->log_error("PlexilBB", "Invalid asynchronous lookup for %s",
		                   state.name().c_str());
		return;
	}
	std::string uid;
	params[0].getValue(uid);

	std::unique_lock<std::mutex> lock(ifs_read_mutex_);
	if (ifs_read_.find(uid) == ifs_read_.end()) {
		logger_->log_warn("PlexilBB", "Invalid asynchronous lookup %s for unknown interface %s",
		                  state.name().c_str(), uid.c_str());
	} else {
		if (subscribed_states_.count(uid) == 0) {
			//logger_->log_debug("PlexilBB", "Updating listener for %s", uid.c_str());
			bbil_add_data_interface(ifs_read_[uid]);
			blackboard_->update_listener(this, BlackBoard::BBIL_FLAG_DATA);
		}
		//logger_->log_debug("PlexilBB", "Subscribe for %s", state.toString().c_str());
		debugMsg("BlackboardAdapter:subscribe", "Subscribe for " << state.toString());
		subscribed_states_.insert({uid, state});
	}
}

/** Unsubscribe from updates.
 * @param state state variable to unsubscribe from
 */
void
BlackboardPlexilAdapter::unsubscribe(const PLEXIL::State& state)
{
	std::vector<PLEXIL::Value> const &params = state.parameters();
	if (params.size() == 0 || params[0].valueType() != PLEXIL::STRING_TYPE) {
		logger_->log_error("PlexilBB", "Invalid asynchronous lookup for %s",
		                   state.name().c_str());
		return;
	}
	std::string uid;
	params[0].getValue(uid);

	std::unique_lock<std::mutex> lock(ifs_read_mutex_);
	if (ifs_read_.find(uid) == ifs_read_.end()) {
		logger_->log_warn("PlexilBB", "Invalid lookup %s unsubscribe for "
		                  "unknown interface %s", state.name().c_str(), uid.c_str());
	} else {
		//logger_->log_debug("PlexilBB", "Unsubscribe for %s", state.toString().c_str());
		debugMsg("BlackboardAdapter:unsubscribe", "Unsubscribe for " << state.toString());
		auto range = subscribed_states_.equal_range(uid);
		for (auto i = range.first; i != range.second; ++i) {
			if (i->second == state) {
				subscribed_states_.erase(i);
				break;
			}
		}
		if (subscribed_states_.count(uid) == 0) {
			bbil_remove_data_interface(ifs_read_[uid]);
			blackboard_->update_listener(this, BlackBoard::BBIL_FLAG_DATA);
		}
	}
}

void
BlackboardPlexilAdapter::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
	//logger_->log_debug("PlexilBB", "Event for %s", interface->uid());
	interface->read();
	auto range = subscribed_states_.equal_range(interface->uid());
	for (auto i = range.first; i != range.second; ++i) {
		if (i->second.name() == "BB_changed") {
			// we need to issue a read call above (otherwise we would handle the
			// value change with the old value) but this will cause Interface::changed()
			// to always return false. However, since this is the handler for the
			// data changed event, we feel confident to return true as value
			m_execInterface.handleValueChange(i->second, PLEXIL::Value(true));
		} else {
			PLEXIL::StateCacheEntry entry;
			lookupNow(i->second, entry);
			m_execInterface.handleValueChange(i->second, entry.cachedValue()->toValue());
		}
	}
	m_execInterface.notifyOfExternalEvent();
}


/** Perform given command.
 * @param cmd command to execute
 */
void
BlackboardPlexilAdapter::executeCommand(PLEXIL::Command* cmd)
{
	std::string const &name = cmd->getName();

	auto c = commands_.find(name);
	if (c != commands_.end()) {
		c->second(cmd);
	} else {
		warn("NavGraphAdapter:executeCommand: called for unknown"
		     " command " << name);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
	}
}

void
BlackboardPlexilAdapter::bb_open_for_reading(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "BlackboardAdapter:bb_open_for_reading",
	                  {{"type", PLEXIL::STRING_TYPE},
	                   {"id", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string if_type;
	std::string if_id;
	args[0].getValue(if_type);
	args[1].getValue(if_id);

	std::string uid{if_type + "::" + if_id};

	//logger_->log_debug("PlexilBB", "Opening %s for reading", uid.c_str());
	debugMsg("BlackboardAdapter:open", "Open for reading " << uid);

	if (ifs_read_.find(uid) == ifs_read_.end()) {
		try {
			ifs_read_[uid] = blackboard_->open_for_reading(if_type.c_str(), if_id.c_str());
			PLEXIL::g_configuration->registerLookupInterface(uid + ".changed", this);
		} catch (Exception &e) {
			logger_->log_warn("PlexilBB", "Failed to open interface %s:%s: %s",
			                  if_type.c_str(), if_id.c_str(), e.what_no_backtrace());
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			m_execInterface.notifyOfExternalEvent();
			return;		
		}
	}

	m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(true));
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

void
BlackboardPlexilAdapter::bb_close(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "BlackboardAdapter:bb_close",
	                  {{"uid", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string uid;
	args[0].getValue(uid);

	//logger_->log_debug("PlexilBB", "Closing %s", uid.c_str());
	debugMsg("BlackboardAdapter:close", "Close " << uid);

	std::unique_lock<std::mutex> lock(ifs_read_mutex_);
	if (ifs_read_.find(uid) == ifs_read_.end()) {
		logger_->log_warn("PlexilBB", "Interface '%s' has not been opened",
		                  uid.c_str());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	blackboard_->close(ifs_read_[uid]);
	ifs_read_.erase(uid);
	
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

void
BlackboardPlexilAdapter::bb_read(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "BlackboardAdapter:bb_read",
	                  {{"uid", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string uid;
	args[0].getValue(uid);

	//logger_->log_debug("PlexilBB", "Reading %s", uid.c_str());
	debugMsg("BlackboardAdapter:read", "Reading " << uid.c_str());

	std::unique_lock<std::mutex> lock(ifs_read_mutex_);
	if (ifs_read_.find(uid) == ifs_read_.end()) {
		logger_->log_warn("PlexilBB", "Interface '%s' has not been opened for reading",
		                  uid.c_str());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	ifs_read_[uid]->read();
	
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

void
BlackboardPlexilAdapter::bb_read_all(PLEXIL::Command* cmd)
{
	//logger_->log_debug("PlexilBB", "Reading all interfaces");
	debugMsg("BlackboardAdapter:read", "Reading all interfaces");

	std::unique_lock<std::mutex> lock(ifs_read_mutex_);
	for (auto & i : ifs_read_) {
		i.second->read();
	}
	
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

void
BlackboardPlexilAdapter::bb_print(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "BlackboardAdapter:bb_print",
	                  {{"uid", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string uid;
	args[0].getValue(uid);

	std::unique_lock<std::mutex> lock(ifs_read_mutex_);
	if (ifs_read_.find(uid) == ifs_read_.end()) {
		logger_->log_warn("PlexilBB", "Interface '%s' has not been opened for reading",
		                  uid.c_str());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	try {
		Interface *i = ifs_read_[uid];

		i->read();
		const Time *t = i->timestamp();

		std::string fact = std::string("(bb-data \"type\" \"") + i->type() + "\"" +
		                   " \"id\" \"" + i->id() + "\"" +
		                   " \"time\" " + StringConversions::to_string(t->get_sec()) + " "
		                   + StringConversions::to_string(t->get_usec()) + ""
		                   + " (. ";

		InterfaceFieldIterator f, f_end = i->fields_end();
		for (f = i->fields(); f != f_end; ++f) {
			std::string value;
			if (f.get_type() == IFT_STRING) {
				value = f.get_value_string();
				std::string::size_type pos = 0;
				while ((pos = value.find("\"", pos)) != std::string::npos) {
					value.replace(pos, 1, "\\\"");
					pos += 2;
				}
				value = std::string("\"") + value + "\"";
			} else if (f.get_type() == IFT_ENUM) {
				value = std::string("\"") + f.get_value_string(" ") + "\"";
			} else {
				value = f.get_value_string(" ");
				std::string::size_type pos;
				while ((pos = value.find(",")) != std::string::npos) {
					value = value.erase(pos, 1);
				}
			}
			if (f.get_length() > 1) {
				fact += std::string(" \"") + f.get_name() + "\" [ " + value + " ]";
			} else {
				fact += std::string(" \"") + f.get_name() + "\" " + value;
			}
		}
		fact += " .))";
		logger_->log_info("PlexilBB", "%s", fact.c_str());
	} catch (Exception &e) {
		logger_->log_warn("PlexilBB", "Failed to print interface '%s': %s",
		                  uid.c_str(), e.what_no_backtrace());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
  }

	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

extern "C" {
	void initFawkesBlackboardAdapter() {
		REGISTER_ADAPTER(BlackboardPlexilAdapter, "FawkesBlackboardAdapter");
	}
}
