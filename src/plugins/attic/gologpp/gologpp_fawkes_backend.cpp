/***************************************************************************
 *  gologpp_fawkes_backend.cpp - Fawkes backend for Golog++
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
 *                   Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "gologpp_fawkes_backend.h"

#include "message_action_executor.h"
#include "print_action_executor.h"
#include "remote_skiller_executor.h"
#include "skiller_action_executor.h"
#include "sleep_action_executor.h"

#include <config/config.h>
#include <golog++/model/activity.h>
#include <golog++/model/transition.h>

namespace fawkes {
namespace gpp {

using namespace gologpp;

/** @class GologppFawkesBackend
 *  A Golog++ backend to get data from and send commands to Fawkes.
 *  The backend currently only provides access to the skiller for action
 *  execution.
 */

/** Constructor.
 *  @param config The configuration to read from
 *  @param cfg_prefix The spec-specific config prefix to use
 *  @param logger The logger to use for log messages
 *  @param blackboard The blackboard to use to access the skiller
 */
GologppFawkesBackend::GologppFawkesBackend(Configuration *config,
                                           std::string    cfg_prefix,
                                           Logger        *logger,
                                           BlackBoard    *blackboard)
: AspectProviderAspect(&dispatcher_inifin_), logger_(logger), blackboard_(blackboard)
{
	// Register RemoteSkillerActionExecutors before the local
	// SkillerActionExecutor. This way, any action that cannot be executed on any
	// remote will be tried locally.
	for (const string &robot :
	     config->get_strings_or_defaults((cfg_prefix + "/agents/names").c_str(), {})) {
		const std::string  agent_prefix = cfg_prefix + "/agents/" + robot;
		const std::string &hostname =
		  config->get_string_or_default((agent_prefix + "/host").c_str(), "localhost");
		const unsigned short int &port =
		  config->get_uint_or_default((agent_prefix + "/port").c_str(), 1910);
		action_dispatcher_.register_executor(std::make_shared<RemoteSkillerActionExecutor>(
		  logger, "robot", robot, hostname, port, config, cfg_prefix));
	}
	if (config->get_bool_or_default((cfg_prefix + "/use_local_skiller").c_str(), true)) {
		action_dispatcher_.register_executor(
		  std::make_shared<SkillerActionExecutor>(logger, blackboard, config, cfg_prefix));
	}
	action_dispatcher_.register_executor(
	  std::make_shared<BBMessageActionExecutor>(logger, blackboard, config, cfg_prefix));
	action_dispatcher_.register_executor(std::make_shared<SleepActionExecutor>(logger));
	action_dispatcher_.register_executor(std::make_shared<PrintActionExecutor>(logger));
}

GologppFawkesBackend::~GologppFawkesBackend()
{
}

/** Preempt the currently running activity.
 *  Determine the right executor and instruct the executor to stop the activity.
 *  @param a The activity to stop
 */
void
GologppFawkesBackend::preempt_activity(shared_ptr<Activity> a)
{
	auto executor = action_dispatcher_.get_executor(a);
	executor->stop(a);
}

/** Get the current time from Fawkes.
 *  @return the current time
 */
gologpp::Clock::time_point
GologppFawkesBackend::time() const noexcept
{
	return gologpp::Clock::time_point{
	  gologpp::Clock::duration{clock->now().in_sec() / gologpp::Clock::duration::period::den}};
}

/** Execute the given activity using a suitable executor.
 *  @param a The activity to start.
 */
void
GologppFawkesBackend::execute_activity(shared_ptr<Activity> a)
{
	auto executor = action_dispatcher_.get_executor(a);
	executor->start(a);
}

} // namespace gpp
} // namespace fawkes
