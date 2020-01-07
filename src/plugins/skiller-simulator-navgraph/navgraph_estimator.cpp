/***************************************************************************
 *  navgraph_estimator.cpp - Estimate skill exec times from the navgraph
 *
 *  Created: Tue 07 Jan 2020 16:35:31 CET 16:35
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

#include "navgraph_estimator.h"

namespace fawkes {
namespace skiller_simulator {

/** @class NavGraphEstimator
 * Estimate the execution time for the skill goto by querying the distance from
 * the navgraph.
 */

/** Constructor.
 * @param navgraph The navgraph to read the node positions from
 * @param pose The Position3DInterface to read the current position from
 */
NavGraphEstimator::NavGraphEstimator(LockPtr<NavGraph> navgraph, Position3DInterface *pose)
: navgraph_(navgraph), pose_(pose)
{
}

bool
NavGraphEstimator::can_execute(const Skill &skill) const
{
	return skill.skill_name == "goto" && navgraph_->node_exists(skill.skill_args.at("place"));
}

float
NavGraphEstimator::get_execution_time(const Skill &skill) const
{
	pose_->read();
	return navgraph_->node(skill.skill_args.at("place"))
	  .distance(pose_->translation(0), pose_->translation(1));
}
} // namespace skiller_simulator
} // namespace fawkes
