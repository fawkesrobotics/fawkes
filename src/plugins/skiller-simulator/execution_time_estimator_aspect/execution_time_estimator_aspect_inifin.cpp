/***************************************************************************
 *  execution_time_estimator_aspect_inifin.cpp - Aspect IniFin
 *
 *  Created: Thu 12 Dec 2019 19:59:57 CET 19:59
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

#include "execution_time_estimator_aspect_inifin.h"

namespace fawkes {
namespace skiller_simulator {

/** @class ExecutionTimeEstimatorsAspectIniFin
 * The Aspect IniFin for the ExecutionTimeEstimatorsAspect.
 */

/** Constructor.
 * @param manager The manager for the time estimators to use in the aspect */
ExecutionTimeEstimatorsAspectIniFin::ExecutionTimeEstimatorsAspectIniFin(
  ExecutionTimeEstimatorManager *manager)
: AspectIniFin("SkillExecutionTimeEstimatorAspect")
{
	execution_time_estimator_manager_ = manager;
}

/** Destructor. */
ExecutionTimeEstimatorsAspectIniFin::~ExecutionTimeEstimatorsAspectIniFin()
{
}

ExecutionTimeEstimatorsAspect *
ExecutionTimeEstimatorsAspectIniFin::get_aspect(Thread *thread) const
{
	ExecutionTimeEstimatorsAspect *exec_thread =
	  dynamic_cast<ExecutionTimeEstimatorsAspect *>(thread);
	if (!exec_thread) {
		throw CannotInitializeThreadException(
		  "Thread '%s' claims to have the SkillExecutionTimeEstimatorAspect, but RRTI says it has not.",
		  thread->name());
	}
	return exec_thread;
}

/** Initialize the thread with the aspect.
 * @param thread The thread to initialize.
 */
void
ExecutionTimeEstimatorsAspectIniFin::init(Thread *thread)
{
	get_aspect(thread)->init_ExecutionTimeEstimatorsAspect(execution_time_estimator_manager_);
}

/** Finalize the aspect of the given thread.
 * @param thread The thread to finalize.
 */
void
ExecutionTimeEstimatorsAspectIniFin::finalize(Thread *thread)
{
	get_aspect(thread)->finalize_ExecutionTimeEstimatorsAspect();
}

} // namespace skiller_simulator
} // namespace fawkes
