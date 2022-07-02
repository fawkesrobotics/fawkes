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

#include <optional>
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
		bool matches(const Skill &skill) const;

		/** The name of the skill */
		std::string skill_name = "";
		/** A map of the skill's argument keys to argument values */
		std::unordered_map<std::string, std::string> skill_args = {};

	private:
		void parse_args(const std::string &args);
	};

	/** A configurable property that is skill-specific and may have a default value. */
	template <typename T>
	class Property
	{
	public:
		Property(fawkes::Configuration  *config,
		         const std::string      &path,
		         const std::string      &property,
		         const std::optional<T> &default_value = std::nullopt);
		T get_property(const std::string &key) const;
		T get_default_value() const;

		/** Mapping from skill entry id to property value */
		std::map<std::string, T> property_entries;

	private:
		/** Global default value that is used whenever no property entry is found */
		std::optional<T> default_value;
	};

	/** Destructor. */
	virtual ~ExecutionTimeEstimator() = default;

	virtual float get_execution_time(const Skill &skill) = 0;
	virtual bool  can_execute(const Skill &skill);
	virtual std::pair<SkillerInterface::SkillStatusEnum, std::string>
	execute(const Skill &skill)
	{
		return std::make_pair(SkillerInterface::SkillStatusEnum::S_FINAL, "");
	};

protected:
	std::map<std::string, Skill> get_skills_from_config(const std::string &path) const;

	virtual bool can_provide_exec_time(const Skill &skill) const = 0;

	template <typename T>
	T get_property(const Property<T> &property) const;
	/** Config to obtain common configurables */
	Configuration *const config_;
	/** Config prefix of the estimator */
	const std::string cfg_prefix_;
	/** Config estimator-specific speedup factor */
	const float speed_;
	/** Points to the whitelist entry that matches the skill to execute. */
	std::map<std::string, Skill>::const_iterator active_whitelist_entry_;
	/** Whitelist of skills that the estimator is allowed to process */
	const std::map<std::string, Skill> whitelist_;
	/** Blacklist of skills that the estimator must not process */
	const std::map<std::string, Skill> blacklist_;

private:
};

} // namespace fawkes
