
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
GlobalStatePlexilAdapter::GlobalStatePlexilAdapter(PLEXIL::AdapterExecInterface &execInterface)
: InterfaceAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
GlobalStatePlexilAdapter::GlobalStatePlexilAdapter(PLEXIL::AdapterExecInterface &execInterface,
                                                   pugi::xml_node const          xml)
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
	config_ =
	  reinterpret_cast<fawkes::Configuration *>(m_execInterface.getProperty("::Fawkes::Config"));
	logger_ = reinterpret_cast<fawkes::Logger *>(m_execInterface.getProperty("::Fawkes::Logger"));

	if (!config_ || !logger_) {
		warn("GlobalState:initialize: requires FawkesRemoteAdapter or must run in plexil plugin");
		return false;
	}

	cfg_default_adapter_        = false;
	const pugi::xml_node config = getXml();
	for (const auto &c : config.children()) {
		if (strcmp(c.name(), "DefaultLookupAdapter") == 0) {
			cfg_default_adapter_ = true;
			logger_->log_warn("GlobalState",
			                  "Default lookup adapter, allowing on-the-fly state registration");
		}
	}

	std::string cfg_prefix;
	try {
		std::string cfg_spec = config_->get_string("/plexil/spec");
		cfg_prefix           = "/plexil/" + cfg_spec + "/";
	} catch (fawkes::Exception &e) {
		warn("GlobalState:initialize: failed to read config: " << e.what_no_backtrace());
		return false;
	}
	cfg_prefix += "global-states/";

	struct GlobalStateValueConfig
	{
		GlobalStateValueConfig() : arity(0)
		{
		}
		std::string       name;
		PLEXIL::ValueType value_type;
		unsigned int      arity;
		struct StateValue
		{
			std::vector<std::string> args;
			PLEXIL::Value            value;
		};
		std::map<std::string, StateValue> values;
	};
	std::map<std::string, GlobalStateValueConfig> configured_values;
	std::unique_ptr<Configuration::ValueIterator> cfg_item{config_->search(cfg_prefix)};
	while (cfg_item->next()) {
		std::string path = cfg_item->path();

		std::string::size_type start_pos = cfg_prefix.size();
		std::string::size_type slash_pos = path.find("/", start_pos + 1);
		if (slash_pos != std::string::npos) {
			std::string id = path.substr(start_pos, slash_pos - start_pos);

			start_pos        = slash_pos + 1;
			slash_pos        = path.find("/", start_pos);
			std::string what = path.substr(start_pos, slash_pos - start_pos);

			if (what == "name") {
				configured_values[id].name = cfg_item->get_string();
			} else if (what == "type") {
				configured_values[id].value_type = PLEXIL::parseValueType(cfg_item->get_string());
			} else if (what == "arity") {
				configured_values[id].arity = cfg_item->get_uint();
			} else if (what == "values") {
				start_pos            = slash_pos + 1;
				slash_pos            = path.find("/", start_pos);
				std::string value_id = path.substr(start_pos, slash_pos - start_pos);

				start_pos              = slash_pos + 1;
				slash_pos              = path.find("/", start_pos);
				std::string value_what = path.substr(start_pos, slash_pos - start_pos);

				if (value_what == "args") {
					configured_values[id].values[value_id].args = cfg_item->get_strings();
				} // ignore value here, we may not know its type, yet
			}
		}
	}

	for (const auto &c : configured_values) {
		if (c.second.name.empty()) {
			warn("GlobalState:initialize: no name for state at index " << c.first);
			return false;
		}
		if (c.second.value_type == PLEXIL::UNKNOWN_TYPE) {
			warn("GlobalState:initialize: missing or invalid type at index " << c.first);
			return false;
		}
		for (const auto &v : c.second.values) {
			if (v.second.args.size() != c.second.arity) {
				warn("GlobalState:initialize: invalid arity value for state " << c.first << " of "
				                                                              << c.second.name);
				return false;
			}
		}

		if (c.second.values.empty()) {
			PLEXIL::State s(c.second.name);
			values_[s] = std::make_pair(c.second.value_type, PLEXIL::Value());
		} else {
			for (const auto &vc : c.second.values) {
				PLEXIL::State s(c.second.name, vc.second.args.size());
				for (size_t i = 0; i < vc.second.args.size(); ++i) {
					s.setParameter(i, vc.second.args[i]);
				}

				PLEXIL::Value v;
				std::string   conf_path = cfg_prefix + c.first + "/values/" + vc.first + "/value";
				if (config_->exists(conf_path)) {
					switch (c.second.value_type) {
					case PLEXIL::STRING_TYPE: v = config_->get_string(conf_path); break;
					case PLEXIL::INTEGER_TYPE: v = config_->get_int(conf_path); break;
					case PLEXIL::REAL_TYPE: v = config_->get_float(conf_path); break;
					case PLEXIL::BOOLEAN_TYPE: v = config_->get_bool(conf_path); break;
					default:
						warn("GlobalState:initialize: Unsupported value type "
						     << PLEXIL::valueTypeName(c.second.value_type) << " for " << s.toString() << " at "
						     << conf_path);
						return false;
					}
				}
				values_[s] = std::make_pair(c.second.value_type, v);
			}
		}
	}

	for (const auto &v : values_) {
		logger_->log_debug("GlobalState",
		                   "Registering value %s=%s",
		                   v.first.toString().c_str(),
		                   v.second.second.valueToString().c_str());
		PLEXIL::g_configuration->registerLookupInterface(v.first.name(), this);
	}

	namespace p = std::placeholders;
	commands_   = {
    {"global_set_int",
     std::bind(&GlobalStatePlexilAdapter::global_set_value, this, p::_1, PLEXIL::INTEGER_TYPE)},
    {"global_set_real",
     std::bind(&GlobalStatePlexilAdapter::global_set_value, this, p::_1, PLEXIL::REAL_TYPE)},
    {"global_set_bool",
     std::bind(&GlobalStatePlexilAdapter::global_set_value, this, p::_1, PLEXIL::BOOLEAN_TYPE)},
    {"global_set_string",
     std::bind(&GlobalStatePlexilAdapter::global_set_value, this, p::_1, PLEXIL::STRING_TYPE)},
    {"global_set_value",
     std::bind(&GlobalStatePlexilAdapter::global_set_value, this, p::_1, PLEXIL::UNKNOWN_TYPE)},
    {"global_print_all", std::bind(&GlobalStatePlexilAdapter::global_print_all, this, p::_1)},
  };

	for (const auto &c : commands_) {
		PLEXIL::g_configuration->registerCommandInterface(c.first, this);
	}

	// For some reason having the respective XML tag is not enough by itself
	if (cfg_default_adapter_) {
		PLEXIL::g_configuration->defaultRegisterAdapter(this);
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
GlobalStatePlexilAdapter::executeCommand(PLEXIL::Command *cmd)
{
	std::string const &name = cmd->getName();

	auto c = commands_.find(name);
	if (c != commands_.end()) {
		c->second(cmd);
	} else {
		warn("GlobalState:executeCommand: called for unknown"
		     " command "
		     << name);
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
GlobalStatePlexilAdapter::lookupNow(PLEXIL::State const &    state,
                                    PLEXIL::StateCacheEntry &cache_entry)
{
	if (values_.find(state) == values_.end()) {
		cache_entry.setUnknown();
		return;
	}

	//printf("Returning %s = %s\n", state.toString().c_str(), values_.at(state).second.valueToString().c_str());
	cache_entry.update(values_.at(state).second);
}

/** Subscribe to updates for given state.
 * @param state state variable to subscribe for
 */
void
GlobalStatePlexilAdapter::subscribe(const PLEXIL::State &state)
{
	subscribed_states_.insert(state);
}

/** Unsubscribe from updates.
 * @param state state variable to unsubscribe from
 */
void
GlobalStatePlexilAdapter::unsubscribe(const PLEXIL::State &state)
{
	subscribed_states_.erase(state);
}

void
GlobalStatePlexilAdapter::global_set_value(PLEXIL::Command *cmd, PLEXIL::ValueType value_type)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (value_type != PLEXIL::UNKNOWN_TYPE) {
		if (!verify_args(args,
		                 "GlobalState:global_set_value",
		                 {{"name", PLEXIL::STRING_TYPE}, {"value", value_type}})) {
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			m_execInterface.notifyOfExternalEvent();
			return;
		}
	} else {
		if (args.size() < 2) {
			warn("GlobalState:global_set_value: Command requires at least 2 arguments, got "
			     << args.size());
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			m_execInterface.notifyOfExternalEvent();
			return;
		}
		for (size_t i = 0; i < args.size() - 1; ++i) {
			if (args[i].valueType() != PLEXIL::STRING_TYPE) {
				warn("GlobalState:global_set_value: "
				     << " argument " << i << " expected to be of type "
				     << "String, but is of type " << PLEXIL::valueTypeName(args[i].valueType()));
				m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
				m_execInterface.notifyOfExternalEvent();
				return;
			}
		}
	}

	std::string name;
	args[0].getValue(name);

	// first is name, last is value, everything in between are params
	size_t num_args = args.size() - 2;

	PLEXIL::State s(name, num_args);
	for (size_t i = 0; i < args.size() - 2; ++i) {
		s.setParameter(i, args[i + 1]);
	}

	if (values_.find(s) == values_.end()) {
		if (cfg_default_adapter_) {
			if (args.back().valueType() != PLEXIL::UNKNOWN_TYPE) {
				logger_->log_debug("GlobalState",
				                   "Adding state %s (value type %s)",
				                   s.toString().c_str(),
				                   PLEXIL::valueTypeName(args.back().valueType()).c_str());
				values_[s] = std::make_pair(args.back().valueType(), args.back());
			}
		} else {
			warn("GlobalState:global_set_value: called for unknown state " << s.toString()
			                                                               << " and not default adapter");
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			m_execInterface.notifyOfExternalEvent();
			return;
		}
	}

	if (args.back().valueType() != values_[s].first) {
		warn("GlobalState:global_set_value: state "
		     << s.toString() << " is of type " << PLEXIL::valueTypeName(values_[s].first)
		     << ", but called "
		     << "with " << PLEXIL::valueTypeName(args.back().valueType()));
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	logger_->log_debug("GlobalState",
	                   "Setting %s = %s",
	                   s.toString().c_str(),
	                   args.back().valueToString().c_str());
	values_[s].second = args.back();

	if (subscribed_states_.find(s) != subscribed_states_.end()) {
		m_execInterface.handleValueChange(s, args.back());
	}

	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

void
GlobalStatePlexilAdapter::global_print_all(PLEXIL::Command *cmd)
{
	logger_->log_info("GlobalState", "Current globals");
	for (const auto &v : values_) {
		logger_->log_info("GlobalState",
		                  "%-40s %-10s %s",
		                  v.first.toString().c_str(),
		                  PLEXIL::valueTypeName(v.second.first).c_str(),
		                  v.second.second.valueToString().c_str());
	}
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

extern "C" {
void
initGlobalState()
{
	REGISTER_ADAPTER(GlobalStatePlexilAdapter, "GlobalState");
}
}
