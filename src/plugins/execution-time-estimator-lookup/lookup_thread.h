/***************************************************************************
 *  lookup_thread.h - Get skill exec times from db lookups
 *
 *  Created: Tue 24 Mar 2020 09:40:18 CET 09:40
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

#pragma once

#include "lookup_estimator.h"

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <execution_time_estimator/aspect/execution_time_estimator.h>
#include <plugins/mongodb/aspect/mongodb.h>

class ExecutionTimeEstimatorLookupEstimatorThread : public fawkes::Thread,
                                                    public fawkes::LoggingAspect,
                                                    public fawkes::ConfigurableAspect,
                                                    public fawkes::MongoDBAspect,
                                                    public fawkes::ExecutionTimeEstimatorsAspect
{
public:
	ExecutionTimeEstimatorLookupEstimatorThread();
	void init();
	void finalize();

private:
	constexpr static char cfg_prefix_[] = "plugins/execution-time-estimator/lookup/";
	std::shared_ptr<fawkes::LookupEstimator> estimator_;
};
