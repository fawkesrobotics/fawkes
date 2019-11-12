/***************************************************************************
 *  message_action_executor.cpp - Action executor for blackboard messages
 *
 *  Created: Wed 30 Oct 2019 13:06:51 CET 13:06
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

#include "message_action_executor.h"

#include "utils.h"

#include <blackboard/blackboard.h>
#include <config/config.h>
#include <golog++/model/activity.h>
#include <logging/logger.h>

#include <algorithm>

namespace fawkes {
namespace gpp {

/** @class BBMessageActionExecutor
 * A Golog++ action executor that sends a message to a blackboard interface.
 * @author Till Hofmann
 */

/** Constructor.
 * Initializes the executor with all the action mappings specified in the config.
 * @param logger A logger to use for logging messages
 * @param blackboard The blackboard to use to send messages to
 * @param config The configuration to read the mapping from
 * @param cfg_prefix The config prefix to use for the message mapping
 */
BBMessageActionExecutor::BBMessageActionExecutor(Logger *           logger,
                                                 BlackBoard *       blackboard,
                                                 Configuration *    config,
                                                 const std::string &cfg_prefix)
: ActionExecutor(logger), blackboard_(blackboard), config_(config), cfg_prefix_(cfg_prefix)
{
	open_interfaces();
}

/** Destructor.
 * Clean up and close all interfaces.
 */
BBMessageActionExecutor::~BBMessageActionExecutor()
{
	close_interfaces();
}

void
BBMessageActionExecutor::open_interfaces()
{
	return;
}

void
BBMessageActionExecutor::close_interfaces()
{
	for (auto interface : open_interfaces_) {
		blackboard_->close(interface.second);
	}
}

bool
BBMessageActionExecutor::can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const
{
	return activity->mapped_name() == "send_message";
}

void
BBMessageActionExecutor::start(std::shared_ptr<gologpp::Activity> activity)
{
	if (!can_execute_activity(activity)) {
		throw Exception("Cannot execute activity '%s' with BBMessageActionExecutor",
		                activity->mapped_name().c_str());
	}
	activity->update(gologpp::Transition::Hook::START);
	std::string interface_type =
	  static_cast<std::string>(activity->mapped_arg_value("interface_type"));
	std::string interface_id = static_cast<std::string>(activity->mapped_arg_value("interface_id"));
	std::string message_type = static_cast<std::string>(activity->mapped_arg_value("message_type"));
	if (open_interfaces_.find(interface_id) == open_interfaces_.end()) {
		open_interfaces_[interface_id] =
		  blackboard_->open_for_reading(interface_type.c_str(), interface_id.c_str());
	}
	Interface *interface = open_interfaces_[interface_id];
	auto       msg       = interface->create_message(message_type.c_str());

	for (auto field = msg->fields(); field != msg->fields_end(); field++) {
		if (activity->target()->mapping().is_mapped(field.get_name())) {
			auto value = activity->mapped_arg_value(field.get_name());
			try {
				value_to_field(value, &field);
			} catch (boost::bad_get &e) {
				logger_->log_error("BBMessageActionExecutor",
				                   "Failed to convert value '%s' of field '%s' with type '%s': '%s'",
				                   value.string_representation().c_str(),
				                   field.get_name(),
				                   field.get_typename(),
				                   e.what());
				activity->update(gologpp::Transition::Hook::FAIL);
				return;
			}
		}
	}
	interface->msgq_enqueue(msg);
	activity->update(gologpp::Transition::Hook::FINISH);
	return;
}

void
BBMessageActionExecutor::stop(std::shared_ptr<gologpp::Grounding<gologpp::Action>> activity)
{
	logger_->log_error("BBMessageActionExecutor",
	                   "Cannot stop a message that has already been sent!");
}

} // namespace gpp
} // namespace fawkes
