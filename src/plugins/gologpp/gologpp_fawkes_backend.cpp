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

#include "skiller_action_executor.h"

#include <golog++/model/activity.h>
#include <golog++/model/transition.h>

namespace fawkes_gpp {

using namespace gologpp;
using namespace fawkes;

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
                                           Logger *       logger,
                                           BlackBoard *   blackboard)
: AspectProviderAspect(&dispatcher_inifin_), logger_(logger), blackboard_(blackboard)
{
	action_dispatcher_.register_executor(
	  std::make_shared<SkillerActionExecutor>(logger, blackboard, config, cfg_prefix));
}

GologppFawkesBackend::~GologppFawkesBackend()
{
}

/** Preempt the currently running activity.
 *  Tell all executors to stop the given transition. Only the executor that is
 *  currently executing the respective Activity should stop execution, all
 *  other executors should do nothing.
 *  @param t The transition of the activity to be stopped
 */
void
GologppFawkesBackend::preempt_activity(shared_ptr<Transition> t)
{
	for (auto &executor : action_dispatcher_.get_executors()) {
		executor->stop(t);
	}
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

} // namespace fawkes_gpp
