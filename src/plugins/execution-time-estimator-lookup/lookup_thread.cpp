/***************************************************************************
 *  lookup_thread.cpp - Get skill exec times from db lookups
 *
 *  Created: Tue 24 Mar 2020 09:02:38 CET 09:02
 *  Copyright  2020  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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

#include "lookup_thread.h"

#include "lookup_estimator.h"

/** @class ExecutionTimeEstimatorLookupEstimatorThread
 * Get estimates for skill execution times from samples of a mongodb database.
 */

/** Constructor. */
ExecutionTimeEstimatorLookupEstimatorThread::ExecutionTimeEstimatorLookupEstimatorThread()
: Thread("ExecutionTimeEstimatorLookupEstimatorThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Register the estimator. */
void
ExecutionTimeEstimatorLookupEstimatorThread::init()
{
	estimator_ =
	  std::make_shared<fawkes::LookupEstimator>(mongodb_connmgr, config, cfg_prefix_, logger);
	estimator_->init();
	execution_time_estimator_manager_->register_provider(
	  estimator_, config->get_int_or_default((std::string{cfg_prefix_} + "priority").c_str(), 0));
}

/** Unregister the estimator. */
void
ExecutionTimeEstimatorLookupEstimatorThread::finalize()
{
	execution_time_estimator_manager_->unregister_provider(estimator_);
}
