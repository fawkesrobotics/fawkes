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

/** @class NavGraphEstimator
 * Estimate the execution time for the skill goto by querying the distance from
 * the navgraph.
 */

/** Constructor.
 * @param navgraph The navgraph to read the node positions from
 * @param config The config to read the initial position from
 * @param cfg_prefix The config prefix to use for config parameters
 */
NavGraphEstimator::NavGraphEstimator(LockPtr<NavGraph>  navgraph,
                                     Configuration     *config,
                                     const std::string &cfg_prefix)
: ExecutionTimeEstimator(config, cfg_prefix),
  navgraph_(navgraph),
  source_names_(config_, cfg_prefix_, "start", ""),
  dest_names_(config_, cfg_prefix_, "target")
{
	last_pose_x_ = config->get_float_or_default("plugins/amcl/init_pose_x", 0);
	last_pose_y_ = config->get_float_or_default("plugins/amcl/init_pose_y", 0);
}

bool
NavGraphEstimator::can_provide_exec_time(const Skill &skill) const
{
	const std::string dest_name = get_property(dest_names_);
	return navgraph_->node_exists(skill.skill_args.at(dest_name));
}

float
NavGraphEstimator::get_execution_time(const Skill &skill)
{
	float       current_pose_x = last_pose_x_;
	float       current_pose_y = last_pose_y_;
	std::string source_name    = get_property(source_names_);
	if (source_name != "") {
		const std::string start = skill.skill_args.at(source_name);
		if (navgraph_->node_exists(start)) {
			current_pose_x = navgraph_->node(start).x();
			current_pose_y = navgraph_->node(start).y();
		}
	}
	return navgraph_->node(skill.skill_args.at(get_property(dest_names_)))
	         .distance(current_pose_x, current_pose_y)
	       / speed_;
}

std::pair<SkillerInterface::SkillStatusEnum, std::string>
NavGraphEstimator::end_execute(const Skill &skill)
{
	auto node    = navgraph_->node(skill.skill_args.at("place"));
	last_pose_x_ = node.x();
	last_pose_y_ = node.y();
	return std::make_pair(SkillerInterface::SkillStatusEnum::S_FINAL, "");
}

} // namespace fawkes
