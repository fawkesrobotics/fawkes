/***************************************************************************
 *  remote_message_executor.cpp - Send messages to a remote blackboard
 *
 *  Created: Fri 14 Feb 2020 13:37:11 CET 13:37
 *  Copyright  2020  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "remote_message_executor.h"

#include <blackboard/remote.h>
#include <golog++/model/activity.h>

namespace fawkes {
namespace gpp {

/** @class RemoteBBMessageActionExecutor
 * An ActionExecutor that sends a message to a remote blackboard.
 * The mapping of an activity to a skill works the same way as for local skills.
 * @author Till Hofmann
 * @see BBMessageActionExecutor
 * @see ActionSkillMapping
 */

/** Constructor.
 * Connect to the given remote host to send messages to its blackboard.
 * @param logger The logger instance to use
 * @param agent_param_name The parameter key to use for checking if this action should be executed on this agent
 * @param agent_param_value The name of the remote agent; only execute the action if it matches this agent name
 * @param hostname The remote hostname to connect to
 * @param port The port to connect to
 * @param config The config to read the action mapping from
 * @param cfg_prefix The spec-specific config prefix to use
 */

RemoteBBMessageActionExecutor::RemoteBBMessageActionExecutor(Logger *           logger,
                                                             const std::string &agent_param_name,
                                                             const std::string &agent_param_value,
                                                             const std::string &hostname,
                                                             unsigned short int port,
                                                             Configuration *    config,
                                                             const std::string &cfg_prefix)
: BBMessageActionExecutor(logger, new RemoteBlackBoard(hostname.c_str(), port), config, cfg_prefix),
  agent_param_name_(agent_param_name),
  agent_param_value_(agent_param_value)
{
	blackboard_owner_ = true;
}

bool
RemoteBBMessageActionExecutor::can_execute_activity(
  std::shared_ptr<gologpp::Activity> activity) const
{
	if (!BBMessageActionExecutor::can_execute_activity(activity)) {
		return false;
	}
	if (!activity->target()->mapping().is_mapped(agent_param_name_)) {
		return false;
	}
	return (static_cast<std::string>(activity->mapped_arg_value(agent_param_name_))
	        == agent_param_value_);
}

/** Get the name of the executor; mainly used for logging.
 * @return The human-readable name of the executor
 */
const char *
RemoteBBMessageActionExecutor::name() const
{
	return "RemoteBBMessageActionExecutor";
}

} // namespace gpp
} // namespace fawkes
