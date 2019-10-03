/***************************************************************************
 *  action_executor_dispatcher.cpp - Dispatch a Golog++ activity to executors
 *
 *  Created: Thu 03 Oct 2019 11:05:01 CEST 11:05
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

#include "action_executor_dispatcher.h"

#include <core/exception.h>
#include <golog++/model/activity.h>

using fawkes::Exception;

namespace fawkes_gpp {

/** @class ActionExecutorDispatcher
 * Dispatch an activity to a number of registered executors.
 * @author Till Hofmann
 */

/** Determine the executor for a given activity.
 * Check all registered executors if any of them can execute the given activity.
 * @param activity The activity to execute.
 * @return The executor that can execute the activity.
 * @throws fawkes::Exception If no suitable executor for the given activity exists.
 */
std::shared_ptr<ActionExecutor>
ActionExecutorDispatcher::get_executor(std::shared_ptr<gologpp::Activity> activity)
{
	for (auto &executor : action_executors_) {
		if (executor->can_execute_activity(activity)) {
			return executor;
		}
	}
	throw Exception(std::string("No known executor for " + activity->mapped_name()).c_str());
}

/** Register a new executor.
 * @param executor The new executor
 */
void
ActionExecutorDispatcher::register_executor(std::shared_ptr<ActionExecutor> executor)
{
	action_executors_.push_back(executor);
}

} // namespace fawkes_gpp
