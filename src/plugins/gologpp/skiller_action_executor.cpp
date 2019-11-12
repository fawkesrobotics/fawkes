/***************************************************************************
 *  skiller_action_executor.cpp - Execute skills for Golog++ activities
 *
 *  Created: Thu 03 Oct 2019 08:58:22 CEST 08:58
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "skiller_action_executor.h"

#include <blackboard/blackboard.h>
#include <config/config.h>
#include <golog++/model/activity.h>
#include <interfaces/SkillerInterface.h>
#include <logging/logger.h>

namespace fawkes {
namespace gpp {

using gologpp::Transition;

/** @class InvalidArgumentException
 * An exception that is thrown if the given arguments do not match the skill's arguments.
 */

/** Constructor.
 * @param format A format string for the format message
 */
InvalidArgumentException::InvalidArgumentException(const char *format, ...) : Exception()
{
	va_list args;
	va_start(args, format);
	append_nolock_va(format, args);
	va_end(args);
}

/** @class SkillerActionExecutor
 * An ActionExecutor that executes an activity using the Skiller.
 * An action is translated to a skill using the skill mapping from the configuration.
 * If the Skiller's status changes, the activity's status is updated accordingly.
 * @author Till Hofmann
 * @see ActionSkillMapping
 */

/** Constructor.
 * Create and initialize the executor, so a subsequent call to start() directly
 * starts executing a skill.
 * @param logger A logger instance to use
 * @param blackboard The blackboard to use to connect to the SkillerInterface
 * @param config The config to read the skill mapping from
 * @param cfg_prefix The spec-specific config prefix to use
 */
SkillerActionExecutor::SkillerActionExecutor(Logger *           logger,
                                             BlackBoard *       blackboard,
                                             Configuration *    config,
                                             const std::string &cfg_prefix)
: ActionExecutor(logger),
  BlackBoardInterfaceListener("Golog++SkillerActionExecutor"),
  blackboard_(blackboard),
  config_(config),
  cfg_prefix_(cfg_prefix)
{
	try {
		skiller_if_ = blackboard_->open_for_reading<SkillerInterface>("Skiller");
	} catch (Exception &e) {
		logger_->log_error(name(), "Failed to open skiller interface: %s", e.what_no_backtrace());
	}
	bbil_add_data_interface(skiller_if_);
	blackboard_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
	skiller_if_->read();
	if (!skiller_if_->has_writer()) {
		blackboard->unregister_listener(this);
		blackboard->close(skiller_if_);
		throw Exception("No writer for Skiller interface");
	}
	skiller_if_->msgq_enqueue(new SkillerInterface::AcquireControlMessage());
	initialize_action_skill_mapping();
}

void
SkillerActionExecutor::initialize_action_skill_mapping()
{
	std::string                        action_mapping_cfg_path = cfg_prefix_ + "/action-mapping/";
	auto                               cfg_iterator{config_->search(action_mapping_cfg_path)};
	std::map<std::string, std::string> mapping;
	while (cfg_iterator->next()) {
		std::string action_name{
		  std::string(cfg_iterator->path()).substr(action_mapping_cfg_path.length())};
		mapping[action_name] = cfg_iterator->get_as_string();
	}
	action_skill_mapping_ = ActionSkillMapping(mapping);
}

/** Destructor.
 * Close all interfaces and unregister from the blackboard.
 */
SkillerActionExecutor::~SkillerActionExecutor()
{
	skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
	blackboard_->unregister_listener(this);
	blackboard_->close(skiller_if_);
}

/** Check if we can execute the given activity.
 * Check the action skill mapping whether the given action occurs in the mapping.
 * If not, we cannot execute the activity.
 * @param activity An activity to execute
 * @return true iff the given activity can be executed by this executor
 */
bool
SkillerActionExecutor::can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const
{
	return action_skill_mapping_.has_mapping(activity->mapped_name());
}

/** Start the given activity.
 * Instruct the skiller to execute the activity.
 * @param activity the activity to execute
 */
void
SkillerActionExecutor::start(std::shared_ptr<gologpp::Activity> activity)
{
	try {
		skiller_if_->msgq_enqueue(
		  new SkillerInterface::ExecSkillMessage(map_activity_to_skill(activity).c_str()));
		running_activity_ = activity;
	} catch (InvalidArgumentException &e) {
		logger_->log_error(name(), "Failed to start %s: %s", activity->name().c_str(), e.what());
		activity->update(Transition::Hook::FAIL);
	}
}

/** Stop the activity if it is currently running.
 * If the given activity does not match the currently running activity, do nothing.
 * @param activity The activity to stop
 */
void
SkillerActionExecutor::stop(std::shared_ptr<gologpp::Grounding<gologpp::Action>> activity)
{
	if (*running_activity_ == *activity) {
		skiller_if_->msgq_enqueue(new SkillerInterface::StopExecMessage());
		running_activity_.reset();
	}
}

const char *
SkillerActionExecutor::name() const
{
	return "SkillerActionExecutor";
}

std::string
SkillerActionExecutor::map_activity_to_skill(std::shared_ptr<gologpp::Activity> activity)
{
	std::map<std::string, std::string> params;
	for (auto &arg : activity->target()->mapping().arg_mapping()) {
		try {
			params[arg.first] = static_cast<std::string>(activity->mapped_arg_value(arg.first));
		} catch (boost::bad_get &e) {
			throw InvalidArgumentException("Failed to cast parameter %s: %s",
			                               arg.first.c_str(),
			                               e.what());
		}
	}
	std::multimap<std::string, std::string> messages;
	if (!action_skill_mapping_.has_mapping(activity->mapped_name())) {
		throw Exception(std::string("No mapping for action " + activity->mapped_name()).c_str());
	}
	auto mapping{action_skill_mapping_.map_skill(activity->mapped_name(), params, messages)};
	for (auto m = messages.find("ERROR"); m != messages.end();) {
		throw InvalidArgumentException("Error occurred while mapping action '%s': %s",
		                               activity->mapped_name().c_str(),
		                               m->second.c_str());
	}
	for (auto m = messages.find("WARNING"); m != messages.end();) {
		logger_->log_warn(name(),
		                  "Warning occurred while mapping action '%s': %s",
		                  activity->mapped_name().c_str(),
		                  m->second.c_str());
	}
	return mapping;
}

/** Update the status of the activity according to the Skiller status.
 * @param iface The interface that has changed
 */
void
SkillerActionExecutor::bb_interface_data_changed(Interface *iface) throw()
{
	if (!running_activity_) {
		return;
	}
	SkillerInterface *skiller_if = dynamic_cast<SkillerInterface *>(iface);
	if (!skiller_if) {
		return;
	}
	skiller_if->read();
	switch (skiller_if->status()) {
	case SkillerInterface::S_FINAL:
		running_activity_->update(Transition::Hook::FINISH);
		running_activity_.reset();
		break;
	case SkillerInterface::S_FAILED: running_activity_->update(Transition::Hook::FAIL); break;
	case SkillerInterface::S_RUNNING: running_activity_->update(Transition::Hook::START); break;
	default: break;
	}
}

} // namespace gpp
} // namespace fawkes
