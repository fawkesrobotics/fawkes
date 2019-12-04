/***************************************************************************
 *  remote_skiller_executor.cpp - Execute Golog++ actions as skills remotely
 *
 *  Created: Tue 03 Dec 2019 14:33:43 CET 14:33
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

#include "remote_skiller_executor.h"

#include <blackboard/remote.h>
#include <golog++/model/activity.h>

namespace fawkes {
namespace gpp {

/** @class RemoteSkillerActionExecutor
 * An ActionExecutor that executes an activity using a Skiller on a remote.
 * The executor connects to a remote blackboard and instructs the remote to execute the respective skill.
 * The mapping of an activity to a skill works the same way as for local skills.
 * @author Till Hofmann
 * @see SkillerActionExecutor
 * @see ActionSkillMapping
 */

/** Constructor.
 * Connect to the given remote host and use that host's skiller interface.
 * @param logger The logger instance to use
 * @param agent_param_name The parameter key to use for checking if this action should be executed on this agent
 * @param agent_param_value The name of the remote agent; only execute the action if it matches this agent name
 * @param hostname The remote hostname to connect to
 * @param port The port to connect to
 * @param config The config to read the skill mapping from
 * @param cfg_prefix The spec-specific config prefix to use
 */
RemoteSkillerActionExecutor::RemoteSkillerActionExecutor(Logger *           logger,
                                                         const std::string &agent_param_name,
                                                         const std::string &agent_param_value,
                                                         const std::string &hostname,
                                                         unsigned short int port,
                                                         Configuration *    config,
                                                         const std::string &cfg_prefix)
: SkillerActionExecutor(logger, new RemoteBlackBoard(hostname.c_str(), port), config, cfg_prefix),
  agent_param_name_(agent_param_name),
  agent_param_value_(agent_param_value)
{
	blackboard_owner_ = true;
}

RemoteSkillerActionExecutor::~RemoteSkillerActionExecutor()
{
}

bool
RemoteSkillerActionExecutor::can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const
{
	if (!SkillerActionExecutor::can_execute_activity(activity)) {
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
RemoteSkillerActionExecutor::name() const
{
	return "RemoteSkillerActionExecutor";
}

} // namespace gpp
} // namespace fawkes
