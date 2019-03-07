
/***************************************************************************
 *  log_adapter.cpp - PLEXIL adapter for Fawkes' log
 *
 *  Created: Mon Aug 13 15:49:25 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "log_adapter.h"

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <Command.hh>
#include <functional>
#include <plan-utils.hh>

/** @class LoggingPlexilAdapter "log_adapter.h"
 * Plexil adapter to provide logging facilities.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
LoggingPlexilAdapter::LoggingPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface)
: InterfaceAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
LoggingPlexilAdapter::LoggingPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface,
                                           pugi::xml_node const          xml)
: InterfaceAdapter(execInterface, xml)
{
}

/** Destructor. */
LoggingPlexilAdapter::~LoggingPlexilAdapter()
{
}

/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
LoggingPlexilAdapter::initialize()
{
	logger_ = reinterpret_cast<fawkes::Logger *>(m_execInterface.getProperty("::Fawkes::Logger"));

	PLEXIL::g_configuration->registerCommandInterface("print", this);
	PLEXIL::g_configuration->registerCommandInterface("pprint", this);
	PLEXIL::g_configuration->registerCommandInterface("printToString", this);
	PLEXIL::g_configuration->registerCommandInterface("pprintToString", this);
	PLEXIL::g_configuration->registerCommandInterface("to_string", this);

	PLEXIL::g_configuration->registerCommandInterface("log_debug", this);
	PLEXIL::g_configuration->registerCommandInterface("log_info", this);
	PLEXIL::g_configuration->registerCommandInterface("log_warn", this);
	PLEXIL::g_configuration->registerCommandInterface("log_error", this);

	return true;
}

/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
LoggingPlexilAdapter::start()
{
	return true;
}

/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
LoggingPlexilAdapter::stop()
{
	return true;
}

/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
LoggingPlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
LoggingPlexilAdapter::shutdown()
{
	return true;
}

static std::string
v_tostring(const std::vector<PLEXIL::Value> &values, bool pretty)
{
	std::string rv;
	if (pretty) {
		rv = pprintToString(values).valueToString();
	} else {
		rv = printToString(values).valueToString();
	}
	if (rv[rv.size() - 1] == '\n') {
		rv.resize(rv.size() - 1);
	}
	return rv;
}

static PLEXIL::Value
v_tovalue(const std::vector<PLEXIL::Value> &values, bool pretty)
{
	PLEXIL::Value rv;
	if (pretty) {
		rv = pprintToString(values).valueToString();
	} else {
		rv = printToString(values).valueToString();
	}
	return rv;
}

/** Perform given command.
 * @param cmd command to execute
 */
void
LoggingPlexilAdapter::executeCommand(PLEXIL::Command *cmd)
{
	std::map<std::string, std::function<void(const std::vector<PLEXIL::Value> &)>> mapping_norv = {
	  {"print",
	   [this](const std::vector<PLEXIL::Value> &values) {
		   this->logger_->log_info("Plexil", "%s", v_tostring(values, false).c_str());
	   }},
	  {"pprint",
	   [this](const std::vector<PLEXIL::Value> &values) {
		   this->logger_->log_info("Plexil", "%s", v_tostring(values, true).c_str());
	   }},
	  {"log_debug",
	   [this](const std::vector<PLEXIL::Value> &values) {
		   this->logger_->log_debug("Plexil", "%s", v_tostring(values, true).c_str());
	   }},
	  {"log_info",
	   [this](const std::vector<PLEXIL::Value> &values) {
		   this->logger_->log_info("Plexil", "%s", v_tostring(values, true).c_str());
	   }},
	  {"log_warn",
	   [this](const std::vector<PLEXIL::Value> &values) {
		   this->logger_->log_warn("Plexil", "%s", v_tostring(values, true).c_str());
	   }},
	  {"log_error", [this](const std::vector<PLEXIL::Value> &values) {
		   this->logger_->log_error("Plexil", "%s", v_tostring(values, true).c_str());
	   }}};

	std::map<std::string, std::function<PLEXIL::Value(const std::vector<PLEXIL::Value> &)>>
	  mapping_rv = {
	    {"printToString",
	     [](const std::vector<PLEXIL::Value> &values) { return v_tovalue(values, false); }},
	    {"pprintToString",
	     [](const std::vector<PLEXIL::Value> &values) { return v_tovalue(values, true); }},
	    {"to_string",
	     [](const std::vector<PLEXIL::Value> &values) { return v_tovalue(values, false); }},
	  };

	const auto &entry_norv = mapping_norv.find(cmd->getName());
	const auto &entry_rv   = mapping_rv.find(cmd->getName());
	if (entry_norv != mapping_norv.end()) {
		entry_norv->second(cmd->getArgValues());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	} else if (entry_rv != mapping_rv.end()) {
		m_execInterface.handleCommandReturn(cmd, entry_rv->second(cmd->getArgValues()));
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	} else {
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
	}

	m_execInterface.notifyOfExternalEvent();
}

/** Abort currently running execution.
 * @param cmd command to abort
 */
void
LoggingPlexilAdapter::invokeAbort(PLEXIL::Command *cmd)
{
	m_execInterface.handleCommandAbortAck(cmd, false);
	m_execInterface.notifyOfExternalEvent();
}

extern "C" {
void
initFawkesLoggingAdapter()
{
	REGISTER_ADAPTER(LoggingPlexilAdapter, "FawkesLoggingAdapter");
}
}
