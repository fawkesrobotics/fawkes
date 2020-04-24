/***************************************************************************
 *  config_estimator.cpp - Read estimated execution time from config
 *
 *  Created: Sun 22 Dec 2019 17:18:10 CET 17:18
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

#include "config_estimator.h"

#include <utils/misc/string_split.h>

#include <memory>

namespace fawkes {

/** @class ConfigExecutionTimeEstimator
 * Get a static estimate for the skill execution time from the config.
 * This simply reads the execution time from the config. It only considers
 * the skill name, the arguments are ignored.
 */

/** Constructor
 * @param config The config to read from
 * @param cfg_prefix The config prefix to read from
 */

ConfigExecutionTimeEstimator::ConfigExecutionTimeEstimator(Configuration *    config,
                                                           const std::string &cfg_prefix)
: ExecutionTimeEstimator(config, cfg_prefix), exec_times_(get_exec_times_from_config())
{
}

/** Load execution time from config
 * @param path_suffix The suffix under which the skill specifications are found
 * @return map from skill entry id to execution time
 */
std::map<std::string, float>
ConfigExecutionTimeEstimator::get_exec_times_from_config(const std::string &path_suffix) const
{
	const int                                     ID       = 0;
	const int                                     PROPERTY = 1;
	std::unique_ptr<Configuration::ValueIterator> it(
	  config_->search((cfg_prefix_ + path_suffix).c_str()));
	std::map<std::string, float> res;
	while (it->next()) {
		std::vector<std::string> skill_property =
		  str_split(std::string(it->path()).substr((cfg_prefix_ + path_suffix).size()));
		if (skill_property.size() != 2) {
			break;
		}
		if (skill_property[PROPERTY] == "time") {
			res[skill_property[ID]] += it->get_float();
		}
	}
	return res;
}

bool
ConfigExecutionTimeEstimator::can_provide_exec_time(const Skill &skill)
{
	if (active_whitelist_entry_ == whitelist_.end()) {
		return config_->exists(cfg_prefix_ + "default");
	} else {
		return exec_times_.find(active_whitelist_entry_->first) != exec_times_.end() || config_->exists(cfg_prefix_ + "default");
	}
}

float
ConfigExecutionTimeEstimator::get_execution_time(const Skill &skill)
{
	if (auto curr_exec_time = exec_times_.find(active_whitelist_entry_->first) == exec_times_.end()) {
    return config_->get_float(cfg_prefix_ + "default");
  } else {
    return curr_exec_time->second;
  }
}
} // namespace fawkes
