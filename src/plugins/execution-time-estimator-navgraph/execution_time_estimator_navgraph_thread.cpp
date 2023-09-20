/***************************************************************************
 *  execution_time_estimator_navgraph_thread.cpp - Skill exec times from navgraph
 *
 *  Created: Tue 07 Jan 2020 16:02:38 CET 16:02
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

#include "execution_time_estimator_navgraph_thread.h"

#include "navgraph_estimator.h"

#include <interfaces/Position3DInterface.h>

/** @class ExecutionTimeEstimatorNavgraphThread
 * Get estimates for skill execution times from the navgraph.
 */

constexpr char ExecutionTimeEstimatorNavgraphThread::cfg_prefix_[];

/** Constructor. */
ExecutionTimeEstimatorNavgraphThread::ExecutionTimeEstimatorNavgraphThread()
: Thread("ExecutionTimeEstimatorNavgraphThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Register the estimator. */
void
ExecutionTimeEstimatorNavgraphThread::init()
{
	estimator_ =
	  std::make_shared<fawkes::NavGraphEstimator>(navgraph, config, cfg_prefix_, blackboard);
	execution_time_estimator_manager_->register_provider(
	  estimator_, config->get_int_or_default((std::string{cfg_prefix_} + "priority").c_str(), 0));
}

/** Unregister the estimator. */
void
ExecutionTimeEstimatorNavgraphThread::finalize()
{
	execution_time_estimator_manager_->unregister_provider(estimator_);
	estimator_.reset();
}
