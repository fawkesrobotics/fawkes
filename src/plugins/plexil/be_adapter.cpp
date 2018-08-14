
/***************************************************************************
 *  be_adapter.cpp - PLEXIL adapter for the Behavior Engine
 *
 *  Created: Tue Aug 14 15:23:21 2018
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

#include "be_adapter.h"

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <Command.hh>

using namespace fawkes;

/** @class BehaviorEnginePlexilAdapter "be_adapter.h"
 * Plexil adapter to provide access to the Behavior Engine.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
BehaviorEnginePlexilAdapter::BehaviorEnginePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface)
: InterfaceAdapter(execInterface),
  BlackBoardInterfaceListener("PlexilBE")
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
BehaviorEnginePlexilAdapter::BehaviorEnginePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
                                               pugi::xml_node const xml)
: InterfaceAdapter(execInterface, xml),
  BlackBoardInterfaceListener("PlexilBE")
{
}

/** Destructor. */
BehaviorEnginePlexilAdapter::~BehaviorEnginePlexilAdapter()
{
}


/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
BehaviorEnginePlexilAdapter::initialize()
{
	logger_     = reinterpret_cast<fawkes::Logger *>(m_execInterface.getProperty("::Fawkes::Logger"));
	blackboard_ = reinterpret_cast<fawkes::BlackBoard *>(m_execInterface.getProperty("::Fawkes::BlackBoard"));

	try {
		skiller_if_ = blackboard_->open_for_reading<SkillerInterface>("Skiller");

		bbil_add_data_interface(skiller_if_);
		blackboard_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
	} catch (Exception &e) {
		logger_->log_error("PlexilBE", "Failed to open skiller interface: %s",
		                   e.what_no_backtrace());
		return false;
	}

	skill_msgid_ = 0;
	current_cmd_ = nullptr;

	PLEXIL::g_configuration->registerCommandInterface("skill_call", this);

	return true;
}


/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
BehaviorEnginePlexilAdapter::start()
{
	skiller_if_->read();
	if (! skiller_if_->has_writer()) {
		logger_->log_error("PlexilBE", "No writer for skiller interface");
		return false;
	}
	if (skiller_if_->exclusive_controller() != skiller_if_->serial()) {
		SkillerInterface::AcquireControlMessage *msg =
		  new SkillerInterface::AcquireControlMessage(/* steal control */ true);
		skiller_if_->msgq_enqueue(msg);
	}
	return true;
}


/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
BehaviorEnginePlexilAdapter::stop()
{
	return true;
}


/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
BehaviorEnginePlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
BehaviorEnginePlexilAdapter::shutdown()
{
	blackboard_->unregister_listener(this);
	return true;
}

std::string
BehaviorEnginePlexilAdapter::format_skillstring(const std::vector<PLEXIL::Value>& values)
{
	std::string rv;
	if (values.size() % 2 == 0) {
		logger_->log_warn("PlexilBE", "Malformed skill call, must be 'skillname argname0 argvalue1...'");
	} else if (values.size() > 0) {
		rv = values[0].valueToString() + "{";
		bool first = true;
		for (size_t i = 1; i < values.size() - 1; i += 2) {
			if (! first) {
				rv += ", ";
			} else {
				first = false;
			}

			rv += values[i].valueToString() + "=";
			if (values[i+1].valueType() == PLEXIL::STRING_TYPE) {
				rv += "\"" + values[i+1].valueToString() + "\"";
			} else {
				rv += values[i+1].valueToString();
			}
		}
		rv += "}";
	}

	return rv;
}


/** Perform given command.
 * @param cmd command to execute
 */
void
BehaviorEnginePlexilAdapter::executeCommand(PLEXIL::Command * cmd)
{
	std::lock_guard<std::mutex> lock(exec_mutex_);
	
	if (cmd->getName() == "skill_call") {
		std::string skill_string = format_skillstring(cmd->getArgValues());

		SkillerInterface::ExecSkillMessage *msg =
		  new SkillerInterface::ExecSkillMessage(skill_string.c_str());
		msg->ref();

		skiller_if_->msgq_enqueue(msg);

    skill_msgid_  = msg->id();
    skill_string_ = skill_string;
    current_cmd_  = cmd;

    msg->unref();

    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SENT_TO_SYSTEM);
	}
  m_execInterface.notifyOfExternalEvent();
}


/** Abort currently running execution.
 * @param cmd command to abort
 */
void
BehaviorEnginePlexilAdapter::invokeAbort(PLEXIL::Command *cmd)
{
	if (current_cmd_) {
		try {
			skiller_if_->msgq_enqueue(new SkillerInterface::StopExecMessage());
		} catch (Exception &e) {}
		current_cmd_ = nullptr;
		m_execInterface.handleCommandAbortAck(cmd, false);
		m_execInterface.notifyOfExternalEvent();
	}
}

void
BehaviorEnginePlexilAdapter::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
	std::lock_guard<std::mutex> lock(exec_mutex_);
	skiller_if_->read();
	if (current_cmd_) {
		if (skiller_if_->msgid() == skill_msgid_) {
			switch (skiller_if_->status()) {
			case SkillerInterface::S_FINAL:
				m_execInterface.handleCommandAck(current_cmd_, PLEXIL::COMMAND_SUCCESS);
				current_cmd_ = nullptr;
				break;
			case SkillerInterface::S_FAILED:
				m_execInterface.handleCommandAck(current_cmd_, PLEXIL::COMMAND_FAILED);
				current_cmd_ = nullptr;
				break;
			default:
				if (current_cmd_->getCommandHandle() == PLEXIL::COMMAND_SENT_TO_SYSTEM) {
					m_execInterface.handleCommandAck(current_cmd_, PLEXIL::COMMAND_RCVD_BY_SYSTEM);
				}
			}
			m_execInterface.notifyOfExternalEvent();
		}
	}
}

extern "C" {
	void initBehaviorEngineAdapter() {
		REGISTER_ADAPTER(BehaviorEnginePlexilAdapter, "BehaviorEngineAdapter");
	}
}
