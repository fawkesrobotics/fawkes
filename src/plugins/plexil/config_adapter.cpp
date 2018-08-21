
/***************************************************************************
 *  config_adapter.cpp - PLEXIL adapter for protobuf_comm
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

#include "config_adapter.h"

#include "utils.h"

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <Command.hh>
#include <InterfaceManager.hh>
#include <StateCacheEntry.hh>

#include <limits>

using namespace fawkes;

/** @class ConfigurationPlexilAdapter "config_adapter.h"
 * Plexil adapter to provide access to the Fawkes configuration.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
ConfigurationPlexilAdapter::ConfigurationPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface)
: InterfaceAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
ConfigurationPlexilAdapter::ConfigurationPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
                                                       pugi::xml_node const xml)
: InterfaceAdapter(execInterface, xml)
{
}

/** Destructor. */
ConfigurationPlexilAdapter::~ConfigurationPlexilAdapter()
{
}


/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
ConfigurationPlexilAdapter::initialize()
{
	logger_  = reinterpret_cast<fawkes::Logger *>(m_execInterface.getProperty("::Fawkes::Logger"));
	config_  = reinterpret_cast<fawkes::Configuration *>(m_execInterface.getProperty("::Fawkes::Config"));

	namespace p = std::placeholders;
	commands_ = {
	  {"config_get_int_or_default",    std::bind(&ConfigurationPlexilAdapter::config_get_value_or_default,
	                                             this, p::_1, PLEXIL::INTEGER_TYPE)},
	  {"config_get_real_or_default",   std::bind(&ConfigurationPlexilAdapter::config_get_value_or_default,
	                                             this, p::_1, PLEXIL::REAL_TYPE)},
	  {"config_get_bool_or_default",   std::bind(&ConfigurationPlexilAdapter::config_get_value_or_default,
	                                             this, p::_1, PLEXIL::BOOLEAN_TYPE)},
	  {"config_get_string_or_default", std::bind(&ConfigurationPlexilAdapter::config_get_value_or_default,
	                                             this, p::_1, PLEXIL::STRING_TYPE)},
	  {"config_get_int",               std::bind(&ConfigurationPlexilAdapter::config_get_value,
	                                             this, p::_1, PLEXIL::INTEGER_TYPE)},
	  {"config_get_real",              std::bind(&ConfigurationPlexilAdapter::config_get_value,
	                                             this, p::_1, PLEXIL::REAL_TYPE)},
	  {"config_get_bool",              std::bind(&ConfigurationPlexilAdapter::config_get_value,
	                                             this, p::_1, PLEXIL::BOOLEAN_TYPE)},
	  {"config_get_string",            std::bind(&ConfigurationPlexilAdapter::config_get_value,
	                                             this, p::_1, PLEXIL::STRING_TYPE)},
	  {"config_exists",                std::bind(&ConfigurationPlexilAdapter::config_exists, this, p::_1)},
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
ConfigurationPlexilAdapter::start()
{
	return true;
}


/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
ConfigurationPlexilAdapter::stop()
{
	return true;
}


/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
ConfigurationPlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
ConfigurationPlexilAdapter::shutdown()
{
	return true;
}

/** Perform given command.
 * @param cmd command to execute
 */
void
ConfigurationPlexilAdapter::executeCommand(PLEXIL::Command* cmd)
{
	std::string const &name = cmd->getName();

	auto c = commands_.find(name);
	if (c != commands_.end()) {
		c->second(cmd);
	} else {
		warn("ConfigCommAdapter:executeCommand: called for unknown"
		     " command " << name);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
	}
}


/** Abort currently running execution.
 * @param cmd command to abort
 */
void
ConfigurationPlexilAdapter::invokeAbort(PLEXIL::Command *cmd)
{
	m_execInterface.handleCommandAbortAck(cmd, false);
	m_execInterface.notifyOfExternalEvent();
}


void
ConfigurationPlexilAdapter::config_get_value_or_default(PLEXIL::Command* cmd, PLEXIL::ValueType value_type)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "ConfigCommAdapter:config_get_value_or_default",
	                  {{"path", PLEXIL::STRING_TYPE},
	                   {"default", value_type}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string   path;
	args[0].getValue(path);

	try {
		switch (value_type) {
		case PLEXIL::STRING_TYPE:
			{
				std::string default_value;
				args[1].getValue(default_value);
				std::string v = config_->get_string_or_default(path.c_str(), default_value);
				m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(v));
			}
			break;

		case PLEXIL::REAL_TYPE:
			{
				double default_value;
				args[1].getValue(default_value);

				double v = default_value;
				// do not use "or_default" here since it can only represent float
				try {
					v = config_->get_float(path.c_str());
				} catch (Exception &e) {} // ignored, use default
				m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(v));
			}
			break;

		case PLEXIL::BOOLEAN_TYPE:
			{
				bool default_value;
				args[1].getValue(default_value);
				bool v = config_->get_bool_or_default(path.c_str(), default_value);
				m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(v));
			}
			break;

		case PLEXIL::INTEGER_TYPE:
			{
				int default_value;
				args[1].getValue(default_value);
				if (config_->is_uint(path.c_str())) {
					unsigned int uv = config_->get_uint(path.c_str());
					if (uv > std::numeric_limits<int>::max()) {
						warn("ConfigCommAdapter:config_get_value_or_default:"
						     << " Unsigned integer too large to store in int (" << uv << ")");
						m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
						m_execInterface.notifyOfExternalEvent();
						return;
					}
				}
				int v = config_->get_int_or_default(path.c_str(), default_value);
				m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(v));
			}
		break;

		default:
			// this would only occur when misconfiguring in initialize()
			warn("ConfigCommAdapter:config_get_value_or_default:"
			     << " Unsupported Plexil type " << PLEXIL::valueTypeName(value_type));
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
			m_execInterface.notifyOfExternalEvent();
			return;
		}

		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
		m_execInterface.notifyOfExternalEvent();

	} catch (Exception &e) {
		warn("ConfigCommAdapter:config_get_value_or_default:"
		     << " Failed to get value " << path << " as " << PLEXIL::valueTypeName(value_type)
		     << ": " << e.what_no_backtrace());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
		m_execInterface.notifyOfExternalEvent();
		return;
	}
}


void
ConfigurationPlexilAdapter::config_get_value(PLEXIL::Command* cmd, PLEXIL::ValueType value_type)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "ConfigCommAdapter:config_get_value",
	                  {{"path", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string   path;
	args[0].getValue(path);

	try {
		switch (value_type) {
		case PLEXIL::STRING_TYPE:
			{
				std::string v = config_->get_string(path.c_str());
				m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(v));
			}
			break;

		case PLEXIL::REAL_TYPE:
			{
				float v = config_->get_float(path.c_str());
				m_execInterface.handleCommandReturn(cmd, PLEXIL::Value((double)v));
			}
			break;

		case PLEXIL::BOOLEAN_TYPE:
		{
			bool v = config_->get_bool(path.c_str());
			m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(v));
		}
		break;

		case PLEXIL::INTEGER_TYPE:
			{
				if (config_->is_uint(path.c_str())) {
					unsigned int uv = config_->get_uint(path.c_str());
					if (uv > std::numeric_limits<int>::max()) {
						warn("ConfigCommAdapter:config_get_value:"
						     << " Unsigned integer too large to store in int (" << uv << ")");
						m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
						m_execInterface.notifyOfExternalEvent();
						return;
					}
				}
				int v = config_->get_int(path.c_str());
				m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(v));
			}
			break;

		default:
			// this would only occur when misconfiguring in initialize()
			warn("ConfigCommAdapter:config_get_value:"
			     << " Unsupported Plexil type " << PLEXIL::valueTypeName(value_type));
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
			m_execInterface.notifyOfExternalEvent();
			return;
		}

		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
		m_execInterface.notifyOfExternalEvent();

	} catch (Exception &e) {
		warn("ConfigCommAdapter:config_get_value:"
		     << " Failed to get value " << path << " as " << PLEXIL::valueTypeName(value_type)
		     << ": " << e.what_no_backtrace());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
		m_execInterface.notifyOfExternalEvent();
		return;
	}
}

void
ConfigurationPlexilAdapter::config_exists(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "ConfigCommAdapter:config_exists",
	                  {{"path", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string   path;
	args[0].getValue(path);

	printf("Checking %s\n", path.c_str());
	printf("Checking %i\n", config_->exists(path.c_str()));
	printf("Checking %u\n", config_->get_uint(path.c_str()));
	
	try {
		m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(config_->exists(path.c_str())));
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
		m_execInterface.notifyOfExternalEvent();
	} catch (Exception &e) {
		warn("ConfigCommAdapter:config_exists:"
		     << " Failed to check " << path << ": " << e.what_no_backtrace());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
		m_execInterface.notifyOfExternalEvent();
		return;
	}
}

extern "C" {
	void initFawkesConfigurationAdapter() {
		REGISTER_ADAPTER(ConfigurationPlexilAdapter, "FawkesConfigurationAdapter");
	}
}
