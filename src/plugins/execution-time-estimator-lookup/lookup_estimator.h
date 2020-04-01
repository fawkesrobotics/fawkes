/***************************************************************************
 *  lookup_estimator.h - Estimate skill exec times via lookups from mongodb
 *
 *  Created: Tue 24 Mar 2020 11:18:59 CET 11:18
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

#include <aspect/logging.h>
#include <config/config.h>
#include <core/threading/mutex.h>
#include <execution_time_estimator/aspect/execution_time_estimator.h>
#include <plugins/mongodb/aspect/mongodb_conncreator.h>

#include <string>
#include <vector>

class ExecutionTimeEstimatorLookupEstimatorThread;

namespace fawkes {
class LookupEstimator : public ExecutionTimeEstimator
{
	/// Friend the ExecutionTimeEstimatorLookupEstimatorThread so that only it can access init
	friend class ::ExecutionTimeEstimatorLookupEstimatorThread;

public:
	LookupEstimator(MongoDBConnCreator *mongo_connection_manager,
	                Configuration *     config,
	                Logger *            logger);
	float                             get_execution_time(const Skill &skill) override;
	bool                              can_execute(const Skill &skill) override;
	SkillerInterface::SkillStatusEnum execute(const Skill &skill,
	                                          std::string &error_feedback) override;

private:
	void init();

	MongoDBConnCreator *mongo_connection_manager_;
	Configuration *     config_;
	Logger *            logger_;

	constexpr static char cfg_prefix_[] = "plugins/skiller-simulator/estimators/lookup/";
	const char *          name_         = "LookupEstimator";

	std::vector<std::string> skills_;
	fawkes::Mutex            mutex_;
	mongocxx::client *       mongodb_client_lookup_;

	bool try_by_default_;

	std::string database_;
	std::string collection_;
	std::string skill_name_field_;
	std::string duration_field_;

	std::string                       error_;
	SkillerInterface::SkillStatusEnum outcome_;
};
} // namespace fawkes
