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

#include <string>
#include <unordered_map>
#include <vector>

namespace fawkes {
namespace skiller_simulator {

class ExecutionTimeEstimator
{
public:
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

	virtual float get_execution_time(const Skill &skill) const = 0;
	virtual bool  can_execute(const Skill &skill) const        = 0;
	virtual void  execute(const Skill &skill){};
};

} // namespace skiller_simulator
} // namespace fawkes
