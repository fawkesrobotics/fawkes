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
 * This simply reads the execution time from the config.
 */

/** Constructor
 * @param config The config to read from
 * @param cfg_prefix The config prefix to read from
 */
ConfigExecutionTimeEstimator::ConfigExecutionTimeEstimator(Configuration *    config,
                                                           const std::string &cfg_prefix)
: ExecutionTimeEstimator(config, cfg_prefix),
  exec_times_(config_,
              cfg_prefix_,
              "time",
              config->exists(cfg_prefix + "default")
                ? std::optional<float>(config->get_float(cfg_prefix + "default"))
                : std::nullopt)
{
}

bool
ConfigExecutionTimeEstimator::can_provide_exec_time(const Skill &skill)
{
	if (active_whitelist_entry_ == whitelist_.end()) {
		return config_->exists(cfg_prefix_ + "default");
	} else {
		return exec_times_.property_entries.find(active_whitelist_entry_->first)
		       != exec_times_.property_entries.end();
	}
}

float
ConfigExecutionTimeEstimator::get_execution_time(const Skill &skill)
{
	return get_property(exec_times_) / speed_;
}
} // namespace fawkes
