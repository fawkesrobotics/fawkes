/***************************************************************************
 *  execution_time_estimator_thread.h - Estimate execution times
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

#pragma once

#include <aspect/aspect_provider.h>
#include <aspect/configurable.h>
#include <aspect/execution_time_estimator.h>
#include <aspect/inifins/execution_time_estimator.h>
#include <core/threading/thread.h>

class ExecutionTimeEstimatorsThread : public fawkes::Thread,
                                      public fawkes::ConfigurableAspect,
                                      public fawkes::AspectProviderAspect
{
public:
	ExecutionTimeEstimatorsThread();

	void init() override;

private:
	fawkes::ExecutionTimeEstimatorManager       execution_time_estimator_manager_;
	fawkes::ExecutionTimeEstimatorsAspectIniFin provider_inifin_;
};
