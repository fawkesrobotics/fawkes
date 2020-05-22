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
	                const std::string & cfg_prefix,
	                Logger *            logger);
	float get_execution_time(const Skill &skill) override;
	bool  can_provide_exec_time(const Skill &skill) const override;
	std::pair<SkillerInterface::SkillStatusEnum, std::string> execute(const Skill &skill) override;

private:
	void init();

	MongoDBConnCreator *mongo_connection_manager_;
	Logger *            logger_;

	constexpr static char logger_name_[]      = "LookupEstimator";
	constexpr static char skill_name_field_[] = "name";
	constexpr static char duration_field_[]   = "duration";

	mutable fawkes::Mutex mutex_;
	mongocxx::client *    mongodb_client_lookup_;

	Property<bool> fully_match_args_;
	Property<bool> include_failures_;

	const std::string instance_;
	const std::string database_;
	const std::string collection_;

	const std::map<std::string, bool> skill_match_args;

	std::string                       error_;
	SkillerInterface::SkillStatusEnum outcome_;
};
} // namespace fawkes
