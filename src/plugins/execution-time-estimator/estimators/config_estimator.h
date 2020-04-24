/***************************************************************************
 *  config_estimator.h - Read estimated execution time from config
 *
 *  Created: Sun 22 Dec 2019 17:16:23 CET 17:16
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

#include <config/config.h>
#include <execution_time_estimator/execution_time_estimator.h>

#include <map>

namespace fawkes {
class ConfigExecutionTimeEstimator : public ExecutionTimeEstimator
{
public:
	ConfigExecutionTimeEstimator(Configuration *config, const std::string &cfg_prefix);
	float get_execution_time(const Skill &skill) override;
	bool  can_provide_exec_time(const Skill &skill) override;

private:
	std::map<std::string, float>
	                                   get_exec_times_from_config(const std::string &path_suffix = "whitelist") const;
	constexpr static char              name_[] = "config";
	const std::map<std::string, float> exec_times_;
};
} // namespace fawkes
