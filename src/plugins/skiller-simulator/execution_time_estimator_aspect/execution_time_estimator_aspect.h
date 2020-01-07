/***************************************************************************
 *  execution_time_estimator_aspect.h - Aspect for a running time provider
 *
 *  Created: Thu 12 Dec 2019 14:52:41 CET 14:52
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

#include "../execution_time_estimator.h"

#include <aspect/aspect.h>

#include <memory>
#include <optional>
#include <vector>

namespace fawkes {
namespace skiller_simulator {

class ExecutionTimeEstimatorManager
{
public:
	std::optional<std::shared_ptr<ExecutionTimeEstimator>>
	     get_provider(const std::string &skill_string) const;
	void register_provider(std::shared_ptr<ExecutionTimeEstimator> provider);

private:
	std::vector<std::shared_ptr<ExecutionTimeEstimator>> execution_time_estimators_;
};

class ExecutionTimeEstimatorsAspect : public virtual Aspect
{
public:
	ExecutionTimeEstimatorsAspect();
	void init_ExecutionTimeEstimatorsAspect(ExecutionTimeEstimatorManager *provider_manager);
	void finalize_ExecutionTimeEstimatorsAspect();

protected:
	ExecutionTimeEstimatorManager *execution_time_estimator_manager_;
};

} // namespace skiller_simulator
} // namespace fawkes
