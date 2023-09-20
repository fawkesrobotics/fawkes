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
                                     const std::string &cfg_prefix,
                                     BlackBoard        *blackboard)
: ExecutionTimeEstimator(config, cfg_prefix),
  navgraph_(navgraph),
  source_names_(config_, cfg_prefix_, "start", ""),
  dest_names_(config_, cfg_prefix_, "target"),
  blackboard_(blackboard)
{
	last_pose_x_                 = config->get_float_or_default("plugins/amcl/init_pose_x", 0);
	last_pose_y_                 = config->get_float_or_default("plugins/amcl/init_pose_y", 0);
	std::string cfg_pose_if_name = config->get_string("plugins/amcl/pose_interface_id");
	publish_pose_ =
	  config->get_bool_or_default("plugins/execution-time-estimator/navgraph/pose-update", true);
	if (publish_pose_) {
		pos3d_if_ = blackboard_->open_for_writing<Position3DInterface>(cfg_pose_if_name.c_str());
		pos3d_if_->set_frame(config->get_string("/frames/fixed").c_str());
		double trans[3] = {last_pose_x_, last_pose_y_, 0};
		pos3d_if_->set_translation(trans);
		pos3d_if_->write();
	}
}

NavGraphEstimator::~NavGraphEstimator()
{
	if (publish_pose_) {
		blackboard_->close(pos3d_if_);
	}
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
	std::string start = get_property(source_names_);
	if (start == "") {
		start = navgraph_->closest_node(last_pose_x_, last_pose_y_).name();
	}
	const std::string end      = skill.skill_args.at(get_property(dest_names_));
	float             duration = 0.f;
	if (navgraph_->node_exists(start) && navgraph_->node_exists(end)) {
		NavGraphPath            path      = navgraph_->search_path(start, end);
		NavGraphPath::Traversal traversal = path.traversal();
		while (traversal.next()) {
			if (!traversal.last()) {
				const NavGraphNode &current = traversal.current();
				const NavGraphNode &next    = traversal.peek_next();
				duration += current.distance(next);
			}
		}
	}
	return duration / speed_;
}

void
NavGraphEstimator::update_pose_along_path(const Skill &skill)
{
	std::string start = get_property(source_names_);
	if (start == "") {
		start = navgraph_->closest_node(last_pose_x_, last_pose_y_).name();
	}
	const std::string end = skill.skill_args.at(get_property(dest_names_));
	if (navgraph_->node_exists(start) && navgraph_->node_exists(end)) {
		NavGraphPath            path      = navgraph_->search_path(start, end);
		NavGraphPath::Traversal traversal = path.traversal();
		while (traversal.next()) {
			const NavGraphNode &current  = traversal.current();
			double              trans[3] = {current.x(), current.y(), 0};
			pos3d_if_->set_translation(trans);
			pos3d_if_->write();
			if (!traversal.last()) {
				const NavGraphNode          &next = traversal.peek_next();
				std::chrono::duration<float> duration(current.distance(next) / speed_);
				std::this_thread::sleep_for(duration);
			}
		}
	}
}

void
NavGraphEstimator::start_execute(const Skill &skill)
{
	if (publish_pose_) {
		// Start a thread to update the value
		pose_publisher_ = std::async(&NavGraphEstimator::update_pose_along_path, this, skill);
	}
}

std::pair<SkillerInterface::SkillStatusEnum, std::string>
NavGraphEstimator::end_execute(const Skill &skill)
{
	if (publish_pose_ && pose_publisher_.valid()) {
		pose_publisher_.wait();
	}
	auto node    = navgraph_->node(skill.skill_args.at("place"));
	last_pose_x_ = node.x();
	last_pose_y_ = node.y();
	return std::make_pair(SkillerInterface::SkillStatusEnum::S_FINAL, "");
}

} // namespace fawkes
