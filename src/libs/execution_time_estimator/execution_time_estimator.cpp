/***************************************************************************
 *  execution_time_estimator.cpp - An execution time estimator for skills
 *
 *  Created: Sun 22 Dec 2019 17:41:18 CET 17:41
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

#include "execution_time_estimator.h"

#include <core/exception.h>

#include <cassert>
#include <iostream>
#include <regex>
#include <string>

namespace fawkes {

/** Use the ExecutionTimeEstimator's skill. */
using Skill = ExecutionTimeEstimator::Skill;

/** @class ExecutionTimeEstimator
 * An abstract estimator for the execution time of a skill.
 * Inherit from this class if you want to implement an estimator for a skill or
 * a set of skills.
 *
 * @fn float ExecutionTimeEstimator::get_execution_time(const Skill &skill) const
 * Get the estimated execution time for the given skill string.
 * @param skill The skill object to compute the execution time for.
 * @return The execution time in seconds.
 *
 * @fn bool ExecutionTimeEstimator::can_provide_exec_time(const Skill &skill)
 * Check if this estimator can give an estimate for a given
 * @param skill The skill object to check.
 * @return true if this estimator can give an execution time estimate for the given skill.
 *
 * @fn bool ExecutionTimeEstimator::can_execute(const Skill &skill)
 * Check if this estimator is both allowed and able to give an estimate for a given
 * @param skill The skill object to check.
 * @return true if this estimator can give an execution time estimate for the given skill.
 *
 * @fn SkillerSimulator::SkillStatusEnum ExecutionTimeEstimator::execute(const Skill &skill, std::string &error_feedback) const
 * Let the estimator know that we are executing this skill, so it can apply
 * possible side effects.
 * @param skill The skill to execute
 * @param error_feedback error message that may be produced while simulating execution
 * @return skill status after simulated execution
 */

/** Constructor.
 * Load config values that are common for all executors.
 * @param config configuration to read all values from
 * @param cfg_prefix prefix where the estimator-specific configs are located
 */
ExecutionTimeEstimator::ExecutionTimeEstimator(Configuration *config, const ::std::string &cfg_prefix)
: config_(config),
  cfg_prefix_(cfg_prefix),
  speed_(config->get_float_or_default((cfg_prefix_ + "speed").c_str(), 1)),
  whitelist_(config->get_strings_or_defaults((cfg_prefix_ + "whitelist").c_str(), {})),
  blacklist_(config->get_strings_or_defaults((cfg_prefix_ + "blacklist").c_str(), {}))
{
}

bool
ExecutionTimeEstimator::can_execute(const Skill &skill)
{
	bool allowed_to_execute = false;
	if (whitelist_.empty()) {
		allowed_to_execute = true;
	} else {
		allowed_to_execute =
		  whitelist_.end() != std::find(whitelist_.begin(), whitelist_.end(), skill.skill_name);
	}
	if (!blacklist_.empty()) {
		allowed_to_execute =
		  allowed_to_execute
		  && blacklist_.end() == std::find(blacklist_.begin(), blacklist_.end(), skill.skill_name);
	}
	return allowed_to_execute && can_provide_exec_time(skill);
}

/** Constructor.
 * Create a skill from the skill string.
 * @param skill_string The skill string to create the skill object from.
 */
Skill::Skill(const std::string &skill_string)
{
	std::string skill_no_newlines(skill_string);
	skill_no_newlines.erase(std::remove(skill_no_newlines.begin(), skill_no_newlines.end(), '\n'),
	                        skill_no_newlines.end());
	if (skill_no_newlines.empty()) {
		return;
	}
	const std::regex regex("(\\w+)(?:\\(\\)|\\{(.+)?\\})");
	std::smatch      match;
	if (std::regex_match(skill_no_newlines, match, regex)) {
		assert(match.size() > 1);
		skill_name = match[1];
		if (match.size() > 2) {
			const std::string args = match[2];
			parse_args(args);
		}
	} else {
		throw Exception("Unexpected skill string: '%s'", skill_no_newlines.c_str());
	}
}

void
Skill::parse_args(const std::string &args)
{
	const std::regex skill_args_regex("(?:([^,]+),\\s*)*?([^,]+)");
	std::smatch      args_match;
	if (std::regex_match(args, args_match, skill_args_regex)) {
		const std::regex skill_arg_regex("(\\w+)=(['\"]?)([^'\"]*)\\2\\s*");
		for (auto kv_match = std::next(args_match.begin()); kv_match != args_match.end(); kv_match++) {
			const std::string key_arg = *kv_match;
			std::smatch       m;
			if (std::regex_match(key_arg, m, skill_arg_regex)) {
				skill_args[m[1]] = m[3];
			}
		}
	}
}

} // namespace fawkes
