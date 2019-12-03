/***************************************************************************
 *  sleep_action_executor.cpp - A simple sleep action
 *
 *  Created: Tue 12 Nov 2019 14:09:55 CET 14:09
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

#include "sleep_action_executor.h"

#include <golog++/model/activity.h>
#include <logging/logger.h>

#include <chrono>
#include <thread>

namespace fawkes {
namespace gpp {

/** @class SleepActionExecutor
 * A Golog++ action executor that just sleeps for a certain amount of time.
 * The action executor sleeps asynchronously and sets the activity to finished
 * after the given time.
 * @author Till Hofmann
 */

/** Constructor.
 * Initialize the executor.
 * @param logger A logger to use for logging messages
 */
SleepActionExecutor::SleepActionExecutor(Logger *logger) : ActionExecutor(logger)
{
}

/** Destructor.
 * Notify all running activities to cancel and wait for them before destruction.
 */
SleepActionExecutor::~SleepActionExecutor()
{
	wait_cond_.notify_all();
	for (auto &fut : running_sleeps_) {
		fut.wait();
	}
}

bool
SleepActionExecutor::can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const
{
	return activity->mapped_name() == "sleep";
}

void
SleepActionExecutor::start(std::shared_ptr<gologpp::Activity> activity)
{
	if (!can_execute_activity(activity)) {
		throw Exception("Cannot execute activity '%s' with SleepActionExecutor",
		                activity->mapped_name().c_str());
	}
	activity->update(gologpp::Transition::Hook::START);
	const std::chrono::duration sleep_duration =
	  std::chrono::seconds(static_cast<long>(activity->mapped_arg_value("seconds")));
	running_sleeps_.push_back(std::async(std::launch::async, ([this, sleep_duration, activity] {
		                                     std::unique_lock<std::mutex> lock(cancel_mutex_);
		                                     auto status = wait_cond_.wait_for(lock, sleep_duration);
		                                     if (status == std::cv_status::timeout
		                                         && activity->state()
		                                              == gologpp::Activity::State::RUNNING) {
			                                     activity->update(gologpp::Transition::Hook::FINISH);
		                                     }
	                                     })));
	running_sleeps_.remove_if([&](auto &fut) {
		return fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
	});
}

void
SleepActionExecutor::stop(std::shared_ptr<gologpp::Grounding<gologpp::Action>> activity)
{
}

} // namespace gpp
} // namespace fawkes
