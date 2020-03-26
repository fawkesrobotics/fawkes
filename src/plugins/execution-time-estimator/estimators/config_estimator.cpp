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
: config_(config), cfg_prefix_(cfg_prefix)
{
}

bool
ConfigExecutionTimeEstimator::can_execute(const Skill &skill)
{
	return config_->exists(cfg_prefix_ + "skills/" + skill.skill_name)
	       || config_->exists(cfg_prefix_ + "default");
}

float
ConfigExecutionTimeEstimator::get_execution_time(const Skill &skill)
{
	if (const std::string cfg_path = cfg_prefix_ + "skills/" + skill.skill_name;
	    config_->exists(cfg_path)) {
		return config_->get_float(cfg_path);
	} else if (const std::string default_path = cfg_prefix_ + "default";
	           config_->exists(default_path)) {
		return config_->get_float(default_path);
	} else
		throw Exception("No config value for %s", cfg_path.c_str());
}
} // namespace fawkes
