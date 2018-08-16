
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

#include <utils/misc/map_skill.h>

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <Command.hh>

#include <algorithm>

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
	config_     = reinterpret_cast<fawkes::Configuration *>(m_execInterface.getProperty("::Fawkes::Config"));

	std::string cfg_prefix;
	try {
		std::string cfg_spec = config_->get_string("/plexil/spec");
		cfg_prefix = "/plexil/" + cfg_spec + "/";
	} catch (fawkes::Exception &e) {
		logger_->log_error("PlexilBE", "Failed to read config: %s", e.what_no_backtrace());
		return false;
	}

	skill_msgid_ = 0;
	current_cmd_ = nullptr;

	// Parse adapter configurations
	std::string skills_config_prefix = cfg_prefix + "skills/";
	std::unique_ptr<Configuration::ValueIterator>
	  cfg_item{config_->search(skills_config_prefix)};
	while (cfg_item->next()) {
		std::string path = cfg_item->path();

		std::string::size_type start_pos = skills_config_prefix.size();
		std::string::size_type slash_pos = path.find("/", start_pos + 1);
		if (slash_pos != std::string::npos) {
			std::string id = path.substr(start_pos, slash_pos - start_pos);
	
			start_pos = slash_pos + 1;
			slash_pos = path.find("/", start_pos);
			std::string what = path.substr(start_pos, slash_pos - start_pos);

			if (what == "name") {
				cfg_skills_[id].name = cfg_item->get_string();
			} else if (what == "template") {
				cfg_skills_[id].template_str = cfg_item->get_string();
			} else if (what == "args") {
				start_pos = slash_pos + 1;
				slash_pos = path.find("/", start_pos);
				size_t args_id = stoi(path.substr(start_pos, slash_pos - start_pos));

				// since we do get the values in order, this keeps or grows size only
				cfg_skills_[id].args.resize(args_id + 1);

				start_pos = slash_pos + 1;
				slash_pos = path.find("/", start_pos);
				std::string args_what = path.substr(start_pos, slash_pos - start_pos);

				if (args_what == "type") {
					std::string type_str = cfg_item->get_as_string();
					cfg_skills_[id].args[args_id].type = PLEXIL::UNKNOWN_TYPE;
					if (type_str == "String") {
						cfg_skills_[id].args[args_id].type = PLEXIL::STRING_TYPE;
					} else if (type_str == "Integer") {
						cfg_skills_[id].args[args_id].type = PLEXIL::INTEGER_TYPE;
					} else if (type_str == "Real") {
						cfg_skills_[id].args[args_id].type = PLEXIL::REAL_TYPE;
					} else if (type_str == "Boolean") {
						cfg_skills_[id].args[args_id].type = PLEXIL::BOOLEAN_TYPE;
					} else {
						logger_->log_warn("PlexilBE", "Invalid argument type '%s' for '%s'",
						                  type_str.c_str(), cfg_skills_[id].name.c_str());
					}
				} else if (args_what == "name") {
					cfg_skills_[id].args[args_id].name = cfg_item->get_as_string();
				}
			}
		}
	}

	PLEXIL::g_configuration->registerCommandInterface("skill_call", this);

	std::map<std::string, std::string> mapping;
	
	logger_->log_debug("PlexilBE", "Skills");
	for (const auto &skill_entry : cfg_skills_) {
		const auto &skill = skill_entry.second;
		std::string line = "- " + skill.name + " (";
		bool first = true;
		for (const auto &arg : skill.args) {
			if (! first) {
				line += ", ";
			} else {
				first = false;
			}
			line += PLEXIL::valueTypeName(arg.type) + " " + arg.name;
		}
		line += ") -> " + skill.template_str;
		logger_->log_debug("PlexilBE", "%s", line.c_str());

		mapping[skill.name] = skill.template_str;
		PLEXIL::g_configuration->registerCommandInterface(skill.name, this);
	}

	action_skill_mapping_ = std::make_shared<fawkes::ActionSkillMapping>(mapping);

	return true;
}


/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
BehaviorEnginePlexilAdapter::start()
{
	try {
		skiller_if_ = blackboard_->open_for_reading<SkillerInterface>("Skiller");

		bbil_add_data_interface(skiller_if_);
		blackboard_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
	} catch (Exception &e) {
		logger_->log_error("PlexilBE", "Failed to open skiller interface: %s",
		                   e.what_no_backtrace());
		return false;
	}

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
	blackboard_->close(skiller_if_);
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


std::string
BehaviorEnginePlexilAdapter::map_skillstring(const std::string& name,
                                             const skill_config& skill_config,
                                             const std::vector<PLEXIL::Value>& values)
{
	if (skill_config.args.size() != values.size()) {
		logger_->log_warn("PlexilBE", "Arguments for '%s' do not match spec (got %zu, expected %zu)",
		                  name.c_str(), skill_config.args.size(), values.size());
		return "";
	}
	for (size_t i = 0; i < skill_config.args.size(); ++i) {
		if (skill_config.args[i].type != values[i].valueType()) {
			logger_->log_warn("PlexilBE", "Arguments type mismatch for '%s' of '%s' (got %s, expected %s)",
			                  skill_config.args[i].name.c_str(), name.c_str(),
			                  PLEXIL::valueTypeName(values[i].valueType()).c_str(),
			                  PLEXIL::valueTypeName(skill_config.args[i].type).c_str());
			return "";
		}
	}
	if (! action_skill_mapping_->has_mapping(name)) {
		logger_->log_warn("PlexilBE", "No mapping for action '%s' known", name.c_str());
		return "";
	}

	std::map<std::string, std::string> param_map;
	for (size_t i = 0; i < skill_config.args.size(); ++i) {
		param_map[skill_config.args[i].name] = values[i].valueToString();
	}

	std::multimap<std::string, std::string> messages;
	std::string rv = action_skill_mapping_->map_skill(name, param_map, messages);
	for (auto &m : messages) {
		if (m.first == "WARN") {
			logger_->log_warn("PlexilBE", "%s", m.second.c_str());
		} else if (m.first == "ERROR") {
			logger_->log_error("PlexilBE", "%s", m.second.c_str());
		} else if (m.first == "DEBUG") {
			logger_->log_debug("PlexilBE", "%s", m.second.c_str());
		} else {
			logger_->log_info("PlexilBE", "%s", m.second.c_str());
		}
	}
	return rv;
}

void
BehaviorEnginePlexilAdapter::call_skill(const std::string& skill_string, PLEXIL::Command* cmd)
{
	SkillerInterface::ExecSkillMessage *msg =
	  new SkillerInterface::ExecSkillMessage(skill_string.c_str());
	msg->ref();

	skiller_if_->msgq_enqueue(msg);

	skill_msgid_  = msg->id();
	skill_string_ = skill_string;
	current_cmd_  = cmd;

	msg->unref();
}

/** Perform given command.
 * @param cmd command to execute
 */
void
BehaviorEnginePlexilAdapter::executeCommand(PLEXIL::Command* cmd)
{
	std::lock_guard<std::mutex> lock(exec_mutex_);
	
	if (cmd->getName() == "skill_call") {
		std::string skill_string = format_skillstring(cmd->getArgValues());
		call_skill(skill_string, cmd);
    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SENT_TO_SYSTEM);
	} else {
		std::string name = cmd->getName();
		auto skill_entry = std::find_if(cfg_skills_.begin(), cfg_skills_.end(),
		                                [&name](const auto &e) {
			                                return e.second.name == name;
		                                });
		if (skill_entry != cfg_skills_.end()) {
			std::string skill_string = map_skillstring(name, skill_entry->second, cmd->getArgValues());
			if (! skill_string.empty()) {
				call_skill(skill_string, cmd);
				m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SENT_TO_SYSTEM);
			} else {
				m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			}
		} else {
			logger_->log_warn("PlexilBE", "Called for unknown skill '%s'", name.c_str());
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		}
	}

	m_execInterface.notifyOfExternalEvent();
}


/** Abort currently running execution.
 * @param cmd command to abort
 */
void
BehaviorEnginePlexilAdapter::invokeAbort(PLEXIL::Command *cmd)
{
	logger_->log_warn("PlexilBE", "Aborting %s", cmd->getName().c_str());
	if (current_cmd_) {
		try {
			skiller_if_->msgq_enqueue(new SkillerInterface::StopExecMessage());
		} catch (Exception &e) {}
		current_cmd_ = nullptr;
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
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
				m_execInterface.handleCommandReturn(current_cmd_, PLEXIL::Value(true));
				m_execInterface.handleCommandAck(current_cmd_, PLEXIL::COMMAND_SUCCESS);
				m_execInterface.notifyOfExternalEvent();
				current_cmd_ = nullptr;
				break;
			case SkillerInterface::S_FAILED:
				m_execInterface.handleCommandReturn(current_cmd_, PLEXIL::Value(false));
				m_execInterface.handleCommandAck(current_cmd_, PLEXIL::COMMAND_FAILED);
				m_execInterface.notifyOfExternalEvent();
				current_cmd_ = nullptr;
				break;
			default:
				if (current_cmd_->getCommandHandle() == PLEXIL::COMMAND_SENT_TO_SYSTEM) {
					m_execInterface.handleCommandAck(current_cmd_, PLEXIL::COMMAND_RCVD_BY_SYSTEM);
				}
			}
		}
	}
}

extern "C" {
	void initBehaviorEngineAdapter() {
		REGISTER_ADAPTER(BehaviorEnginePlexilAdapter, "BehaviorEngineAdapter");
	}
}
