
/***************************************************************************
 *  globals_adapter.cpp - PLEXIL adapter for global state
 *
 *  Created: Thu Aug 16 11:06:55 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "globals_adapter.h"

#include "utils.h"

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <Command.hh>
#include <InterfaceManager.hh>
#include <StateCacheEntry.hh>

#include <limits>

using namespace fawkes;

/** @class GlobalStatePlexilAdapter "config_adapter.h"
 * Plexil adapter to provide access to the Fawkes configuration.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
GlobalStatePlexilAdapter::GlobalStatePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface)
: InterfaceAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
GlobalStatePlexilAdapter::GlobalStatePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
                                                       pugi::xml_node const xml)
: InterfaceAdapter(execInterface, xml)
{
}

/** Destructor. */
GlobalStatePlexilAdapter::~GlobalStatePlexilAdapter()
{
}


/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
GlobalStatePlexilAdapter::initialize()
{
	config_     = reinterpret_cast<fawkes::Configuration *>(m_execInterface.getProperty("::Fawkes::Config"));

	if (!config_) {
		warn("GlobalState:initialize: requires FawkesRemoteAdapter or must run in plexil plugin");
		return false;
	}

	std::string cfg_prefix;
	try {
		std::string cfg_spec = config_->get_string("/plexil/spec");
		cfg_prefix = "/plexil/" + cfg_spec + "/";
	} catch (fawkes::Exception &e) {
		warn("GlobalState:initialize: failed to read config: " << e.what_no_backtrace());
		return false;
	}
	cfg_prefix += "global-states/";

	std::map<std::string, std::pair<std::string, PLEXIL::ValueType>> configured_values;
	std::unique_ptr<Configuration::ValueIterator>
	  cfg_item{config_->search(cfg_prefix)};
	while (cfg_item->next()) {
		std::string path = cfg_item->path();

		std::string::size_type start_pos = cfg_prefix.size();
		std::string::size_type slash_pos = path.find("/", start_pos + 1);
		if (slash_pos != std::string::npos) {
			std::string id = path.substr(start_pos, slash_pos - start_pos);

			start_pos = slash_pos + 1;
			slash_pos = path.find("/", start_pos);
			std::string what = path.substr(start_pos, slash_pos - start_pos);

			if (what == "name") {
				configured_values[id].first  = cfg_item->get_string();
			} else if (what == "type") {
				configured_values[id].second = PLEXIL::parseValueType(cfg_item->get_string());
			}
		}
	}

	for (const auto &c: configured_values) {
		if (c.second.first.empty()) {
			warn("GlobalState:initialize: no name for state at index " << c.first);
			return false;
		}
		if (c.second.second == PLEXIL::UNKNOWN_TYPE) {
			warn("GlobalState:initialize: missing or invalid type at index " << c.first);
			return false;
		}
		values_[c.second.first] = std::make_pair(c.second.second, PLEXIL::Value());
	}

	for (const auto &v: values_) {
		printf("Registering value %s\n", v.first.c_str());
		PLEXIL::g_configuration->registerLookupInterface(v.first, this);
	}

	namespace p = std::placeholders;
	commands_ = {
	  {"global_set_int",    std::bind(&GlobalStatePlexilAdapter::global_set_value,
	                                  this, p::_1, PLEXIL::INTEGER_TYPE)},
	  {"global_set_real",   std::bind(&GlobalStatePlexilAdapter::global_set_value,
	                                  this, p::_1, PLEXIL::REAL_TYPE)},
	  {"global_set_bool",   std::bind(&GlobalStatePlexilAdapter::global_set_value,
	                                  this, p::_1, PLEXIL::BOOLEAN_TYPE)},
	  {"global_set_string", std::bind(&GlobalStatePlexilAdapter::global_set_value,
	                                  this, p::_1, PLEXIL::STRING_TYPE)},
	  {"global_set_value",  std::bind(&GlobalStatePlexilAdapter::global_set_value,
	                                  this, p::_1, PLEXIL::UNKNOWN_TYPE)},
	};

	for (const auto &c: commands_) {
		PLEXIL::g_configuration->registerCommandInterface(c.first, this);
	}

	return true;
}


/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
GlobalStatePlexilAdapter::start()
{
	return true;
}


/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
GlobalStatePlexilAdapter::stop()
{
	return true;
}


/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
GlobalStatePlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
GlobalStatePlexilAdapter::shutdown()
{
	return true;
}

/** Perform given command.
 * @param cmd command to execute
 */
void
GlobalStatePlexilAdapter::executeCommand(PLEXIL::Command* cmd)
{
	std::string const &name = cmd->getName();

	auto c = commands_.find(name);
	if (c != commands_.end()) {
		c->second(cmd);
	} else {
		warn("GlobalState:executeCommand: called for unknown"
		     " command " << name);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
	}
}


/** Abort currently running execution.
 * @param cmd command to abort
 */
void
GlobalStatePlexilAdapter::invokeAbort(PLEXIL::Command *cmd)
{
	m_execInterface.handleCommandAbortAck(cmd, false);
	m_execInterface.notifyOfExternalEvent();
}


/** Immediate lookup of value.
 * @param state state variable to lookup
 * @param cache_entry cache entry for retrieved value
 */
void
GlobalStatePlexilAdapter::lookupNow(PLEXIL::State const &state, PLEXIL::StateCacheEntry &cache_entry)
{
	if (values_.find(state.name()) == values_.end()) {
		cache_entry.setUnknown();
		return;
	}

	cache_entry.update(values_[state.name()].second);
}

/** Subscribe to updates for given state.
 * @param state state variable to subscribe for
 */
void
GlobalStatePlexilAdapter::subscribe(const PLEXIL::State& state)
{
	subscribed_states_[state.name()] = state;
}

/** Unsubscribe from updates.
 * @param state state variable to unsubscribe from
 */
void
GlobalStatePlexilAdapter::unsubscribe(const PLEXIL::State& state)
{
	subscribed_states_.erase(state.name());
}


void
GlobalStatePlexilAdapter::global_set_value(PLEXIL::Command* cmd, PLEXIL::ValueType value_type)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "GlobalState:global_set_value",
	                  {{"name", PLEXIL::STRING_TYPE},
	                   {"value", value_type}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string   name;
	args[0].getValue(name);

	if (values_.find(name) == values_.end()) {
		warn("GlobalState:global_set_value: called for unknown state " << name);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	if (args[1].valueType() != values_[name].first) {
		warn("GlobalState:global_set_value: state " << name << " is of type "
		     << PLEXIL::valueTypeName(values_[name].first) << ", but called "
		     << "with " << PLEXIL::valueTypeName(args[1].valueType()));
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	printf("Setting %s = %s\n", name.c_str(), args[1].valueToString().c_str());
	values_[name].second = args[1];

	if (subscribed_states_.find(name) != subscribed_states_.end()) {
		m_execInterface.handleValueChange(subscribed_states_[name], args[1]);
	}

	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

extern "C" {
	void initGlobalState() {
		REGISTER_ADAPTER(GlobalStatePlexilAdapter, "GlobalState");
	}
}
