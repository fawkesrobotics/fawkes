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

namespace fawkes {
namespace gpp {

/** @class ActionExecutorDispatcher
 * Dispatch an activity to a number of registered executors by checking all
 * registered executors subsequently, whether they can execute the given
 * activity. The first suitable executor is used to execute the activity.
 * @author Till Hofmann
 */

/** Determine the executor for a given activity.
 * Check all registered executors if any of them can execute the given activity.
 * @param activity The activity to execute.
 * @return The executor that can execute the activity.
 * @throws Exception If no suitable executor for the given activity exists.
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

/** Get the currently registered executors.
 * @return a vector of currently registered executors.
 */
std::vector<std::shared_ptr<ActionExecutor>>
ActionExecutorDispatcher::get_executors() const
{
	return action_executors_;
}

} // namespace gpp

/** @class GologppDispatcherAspect
 * An aspect that provides access to the Golog++ Action Executor Dispatcher.
 * Use this if you implement an executor for Golog++. Your action executor
 * should register itself by calling
 * gpp::ActionExecutorDispatcher::register_executor().
 * @author Till Hofmann
 * @see gpp::ActionExecutorDispatcher
 * @see gpp::ActionExecutor
 */

/** @var GologppDispatcherAspect::gologpp_dispatcher
 * A pointer to the dispatcher that the aspect provides.
 * Use this dispatcher to register your executor.
 */

/** Constructor. */
GologppDispatcherAspect::GologppDispatcherAspect() : gologpp_dispatcher(nullptr)
{
	add_aspect("GologppDispatcherAspect");
}

/** Init GologppDispatcherAspect.
 * Initialize the aspect with the given dispatcher instance.
 * @param dispatcher The dispatcher to use
 */
void
GologppDispatcherAspect::init_GologppDispatcherAspect(gpp::ActionExecutorDispatcher *dispatcher)
{
	gologpp_dispatcher = dispatcher;
}

/** Finalize the GologppDispatcherAspect.
 */
void
GologppDispatcherAspect::finalize_GologppDispatcherAspect()
{
}

} // namespace fawkes
