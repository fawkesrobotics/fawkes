/***************************************************************************
 *  action_executor.cpp - An abstract action executor for Golog++
 *
 *  Created: Thu 03 Oct 2019 08:56:15 CEST 08:56
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

#include "action_executor.h"

namespace fawkes_gpp {

/** @class ActionExecutor
 * Abstract class to execute a Golog++ activity.
 * Any implementation is expected to give feedback about the execution by
 * directly calling the activity's update function.
 * @author Till Hofmann
 *
 * @fn ActionExecutor::start(std::shared_ptr<gologpp::Activity> activity)
 * Start the given activity.
 * @param activity The activity to execute.
 *
 * @fn ActionExecutor::stop(std::shared_ptr<gologpp::Activity> activity)
 * Stop the given activity.
 * The executor is expected to keep track of the currently executed activity.
 * Only stop the activity if the executor is actually executing it. If the
 * given activity is not executed by this executor, do nothing.
 * @param activity The activity to stop.
 *
 * @fn ActionExecutor::can_execute_activity(std::shared_ptr<gologpp::Activity> activity)
 * Determine if this executor can execute the given activity.
 * @param activity The activity to execute.
 * @return True iff the given activity can be executed by this executor.
 *
 * @var ActionExecutor::running_activity_
 * A pointer to the currently running activity.
 *
 * @var ActionExecutor::logger_
 * The logger to use for logging messages.
 */

/** Constructor of an abstract executor.
 * @param logger The logger to use
 */
ActionExecutor::ActionExecutor(fawkes::Logger *logger) : logger_(logger)
{
}

} // namespace fawkes_gpp
