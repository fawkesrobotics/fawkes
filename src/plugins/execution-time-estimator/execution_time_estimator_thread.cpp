/***************************************************************************
 *  execution_time_estimator_thread.cpp - 
 *
 *  Created: Thu 23 Apr 2020 17:07:11 CEST 17:07
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

#include "execution_time_estimator_thread.h"

#include "estimators/config_estimator.h"

/** @class ExecutionTimeEstimatorsThread
 *  The plugin thread, initializes the aspect.
 *
 *  @author Till Hofmann
 */

constexpr char ExecutionTimeEstimatorsThread::cfg_prefix_[];

ExecutionTimeEstimatorsThread::ExecutionTimeEstimatorsThread()
: Thread("ExecutionTimeEstimatorThread", Thread::OPMODE_WAITFORWAKEUP),
  AspectProviderAspect(&provider_inifin_),
  provider_inifin_(&execution_time_estimator_manager_)
{
}

void
ExecutionTimeEstimatorsThread::init()
{
	execution_time_estimator_manager_.register_provider(
	  std::make_shared<fawkes::ConfigExecutionTimeEstimator>(config, cfg_prefix_), -1);
}
