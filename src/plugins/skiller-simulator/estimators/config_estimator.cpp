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
namespace skiller_simulator {

constexpr char ConfigExecutionTimeEstimator::cfg_prefix_[];

/** @class ConfigExecutionTimeEstimator
 * Get a static estimate for the skill execution time from the config.
 * This simply reads the execution time from the config. It only considers
 * the skill name, the arguments are ignored.
 */

/** Constructor
 * @param config The config to read the execution times from
 */
ConfigExecutionTimeEstimator::ConfigExecutionTimeEstimator(Configuration *config) : config_(config)
{
}

bool
ConfigExecutionTimeEstimator::can_execute(const Skill &skill) const
{
	return config_->exists(cfg_prefix_ + skill.skill_name);
}

float
ConfigExecutionTimeEstimator::get_execution_time(const Skill &skill) const
{
	return config_->get_float(cfg_prefix_ + skill.skill_name);
}
} // namespace skiller_simulator
} // namespace fawkes
