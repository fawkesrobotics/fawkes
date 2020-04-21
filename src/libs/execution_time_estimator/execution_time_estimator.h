/***************************************************************************
 *  execution_time_estimator.h - An execution time estimator for skills
 *
 *  Created: Thu 12 Dec 2019 15:10:06 CET 15:10
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
#include <interfaces/SkillerInterface.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace fawkes {

class ExecutionTimeEstimator
{
public:
	ExecutionTimeEstimator(Configuration *config, const ::std::string &cfg_prefix);
	/** A structured representation of a skill. */
	class Skill
	{
	public:
		Skill(const std::string &skill_string);

		/** The name of the skill */
		std::string skill_name = "";
		/** A map of the skill's argument keys to argument values */
		std::unordered_map<std::string, std::string> skill_args = {};

	private:
		void parse_args(const std::string &args);
	};

	/** Destructor. */
	virtual ~ExecutionTimeEstimator() = default;

	virtual float get_execution_time(const Skill &skill)    = 0;
	virtual bool  can_provide_exec_time(const Skill &skill) = 0;
	bool          can_execute(const Skill &skill);
	virtual SkillerInterface::SkillStatusEnum
	execute(const Skill &skill, std::string &error_feedback)
	{
		return SkillerInterface::SkillStatusEnum::S_FINAL;
	};

protected:
	/** Config to obtain common configurables */
	Configuration *const config_;
	/** config prefix of the estimator */
	const std::string cfg_prefix_;
	/** config estimato-specific speedup factor */
	const float speed_;
	/** whitelist of skills that the estimator is allowed to process */
	const std::vector<std::string> whitelist_;
	/** blacklist of skills that the estimator must not process */
	const std::vector<std::string> blacklist_;

private:
};

} // namespace fawkes
