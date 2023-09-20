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

#include <aspect/blackboard.h>
#include <config/config.h>
#include <execution_time_estimator/aspect/execution_time_estimator.h>
#include <navgraph/navgraph.h>

#include <future>
#include <string>
#include <vector>

namespace fawkes {
class NavGraphEstimator : public ExecutionTimeEstimator
{
public:
	NavGraphEstimator(LockPtr<NavGraph>  navgraph,
	                  Configuration     *config,
	                  const std::string &cfg_prefix,
	                  BlackBoard        *blackboard);
	~NavGraphEstimator();
	float get_execution_time(const Skill &skill) override;
	bool  can_provide_exec_time(const Skill &skill) const override;
	void  start_execute(const Skill &skill) override;
	void  update_pose_along_path(const Skill &skill);
	std::pair<SkillerInterface::SkillStatusEnum, std::string>
	end_execute(const Skill &skill) override;

private:
	LockPtr<NavGraph>            navgraph_;
	float                        last_pose_x_;
	float                        last_pose_y_;
	float                        curr_duration_;
	const Property<std::string>  source_names_;
	const Property<std::string>  dest_names_;
	std::future<void>            pose_publisher_;
	fawkes::Position3DInterface *pos3d_if_;
	BlackBoard                  *blackboard_;
	bool                         publish_pose_;
};
} // namespace fawkes
