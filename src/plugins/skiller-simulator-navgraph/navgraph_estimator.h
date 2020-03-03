/***************************************************************************
 *  navgraph_estimator.h - Estimate skill exec times from the navgraph
 *
 *  Created: Tue 07 Jan 2020 16:18:59 CET 16:18
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

#include "interfaces/Position3DInterface.h"

#include <config/config.h>
#include <navgraph/navgraph.h>
#include <plugins/skiller-simulator/execution_time_estimator.h>

#include <string>
#include <vector>

namespace fawkes {
namespace skiller_simulator {
class NavGraphEstimator : public ExecutionTimeEstimator
{
public:
	NavGraphEstimator(LockPtr<NavGraph> navgraph, Configuration *config);
	float get_execution_time(const Skill &skill) const override;
	bool  can_execute(const Skill &skill) const override;
	void  execute(const Skill &skill) override;

private:
	LockPtr<NavGraph>        navgraph_;
	std::vector<std::string> skills_;
	float                    last_pose_x_;
	float                    last_pose_y_;
	float                    speed_;
};
} // namespace skiller_simulator
} // namespace fawkes
