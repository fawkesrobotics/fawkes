/***************************************************************************
 *  execution_time_estimator_aspect_inifin.h - Aspect INiFin
 *
 *  Created: Thu 12 Dec 2019 19:00:07 CET 19:00
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

#pragma once

#include "execution_time_estimator_aspect.h"

#include <aspect/aspect.h>
#include <aspect/inifins/inifin.h>

namespace fawkes {
namespace skiller_simulator {
class ExecutionTimeEstimatorsAspectIniFin : public virtual AspectIniFin
{
public:
	ExecutionTimeEstimatorsAspectIniFin(ExecutionTimeEstimatorManager *manager);
	virtual ~ExecutionTimeEstimatorsAspectIniFin();
	virtual void init(Thread *thread);
	virtual void finalize(Thread *thread);

private:
	ExecutionTimeEstimatorsAspect *get_aspect(Thread *thread) const;
	ExecutionTimeEstimatorManager *execution_time_estimator_manager_;
};

} // namespace skiller_simulator
} // namespace fawkes
