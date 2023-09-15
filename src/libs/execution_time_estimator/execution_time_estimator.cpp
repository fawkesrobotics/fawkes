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
#include <utils/misc/string_split.h>

#include <cassert>
#include <iostream>
#include <map>
#include <regex>
#include <string>

// implementation workaround for type dependent static_assert
// see https://en.cppreference.com/w/cpp/language/if#Constexpr_If
template <class T>
struct dependent_false : std::false_type
{
};

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
 * @fn bool ExecutionTimeEstimator::can_provide_exec_time(const Skill &skill) const
 * Check if this estimator can give an estimate for a given skill.
 * @param skill The skill object to check.
 * @return true if this estimator can give an execution time estimate for the given skill.
 *
 * @fn bool ExecutionTimeEstimator::can_execute(const Skill &skill)
 * Check if this estimator is both allowed and able to give an estimate for a given skill.
 * @param skill The skill object to check.
 * @return true if this estimator can give an execution time estimate for the given skill.
 *
 * @fn void ExecutionTimeEstimator::start_execute(const Skill &skill) const
 * Let the estimator know that we start executing this skill, so it can apply
 * possible side effects.
 * @param skill The skill to execute
 *
 * @fn std::pair<SkillerInterface::SkillStatusEnum, std::string> ExecutionTimeEstimator::end_execute(const Skill &skill) const
 * Let the estimator know that we are ending the execution of this skill to determine the final status
 * @param skill The skill to execute
 * @return skill status after simulated execution along with an error description in case the skill fails
 *
 * @fn std::map<std::string, Skill> ExecutionTimeEstimator::get_skills_from_config(const std::string &path) const
 * Load skill descriptions from a yaml config. The skills are represented via
 * suffixes /<description-id>/name and /<description-id>/args following the
 * @param path config path under which the skill descriptions are located
 * @return all skills found under the given path sorted by <description-id>.
 *
 * @fn template <typename T> T ExecutionTimeEstimator::get_property(const Property<T> &property) const
 * Get the current property value for \a active_whitelist_entry_.
 * @param property property where the current value should be retrieved from
 * @return property accoring to \a active_whitelist_entry_ or the default value if no whitelist entry is active
 *
 *  @fn bool Skill::matches(const Skill &other) const
 * Check, whether the skill matches another skill description.
 * @param other The skill description that should be matched
 * @return true if all skill args in other are contained in the args of this skill and the skill names match
 *
 * @fn template <typename T> ExecutionTimeEstimator::Property<T>::Property(fawkes::Configuration *config, const std::string & path, const std::string &property, const T &default_value)
 * Constructor.
 * Create a property by reading all values from the config.
 * @param config Config to read form
 * @param path Path under which the config values can be found
 * @param property Property name
 * @param default_value Default value in case values are not specified
 *
 * @fn template <typename T> T ExecutionTimeEstimator::Property<T>::get_default_value() const
 * Get the default value if it is set, otherwise throw an exception
 * @return the default value for the property
 *
 * @fn template <typename T> T ExecutionTimeEstimator::Property<T>::get_property(const std::string &key) const
 * Get the property falue for a given skill.
 * @param key Skill entry id
 * @return Value associated with \a key or the default value, if no skill-specific value can be found
 */

/** Constructor.
 * Load config values that are common for all executors.
 * @param config configuration to read all values from
 * @param cfg_prefix prefix where the estimator-specific configs are located
 */
ExecutionTimeEstimator::ExecutionTimeEstimator(Configuration       *config,
                                               const ::std::string &cfg_prefix)
: config_(config),
  cfg_prefix_(cfg_prefix),
  speed_(config->get_float_or_default((cfg_prefix_ + "speed").c_str(), 1)),
  whitelist_(get_skills_from_config(cfg_prefix_ + "whitelist")),
  blacklist_(get_skills_from_config(cfg_prefix_ + "blacklist"))
{
	assert(speed_ > 0);
}

bool
ExecutionTimeEstimator::can_execute(const Skill &skill)
{
	bool allowed_to_execute = false;
	if (whitelist_.empty()) {
		allowed_to_execute      = true;
		active_whitelist_entry_ = whitelist_.end();
	} else {
		active_whitelist_entry_ =
		  std::find_if(whitelist_.begin(), whitelist_.end(), [skill](const auto &s) {
			  return skill.matches(s.second);
		  });

		allowed_to_execute = active_whitelist_entry_ != whitelist_.end();
	}
	if (!blacklist_.empty()) {
		allowed_to_execute =
		  allowed_to_execute
		  && blacklist_.end()
		       == std::find_if(blacklist_.begin(), blacklist_.end(), [skill](const auto &s) {
			          return skill.matches(s.second);
		          });
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

bool
Skill::matches(const Skill &other) const
{
	bool args_match = true;
	for (const auto &arg : other.skill_args) {
		auto search_arg = skill_args.find(arg.first);
		if (search_arg == skill_args.end()
		    || !std::regex_match(search_arg->second, std::regex(arg.second))) {
			args_match = false;
			break;
		}
	}
	return args_match && skill_name == other.skill_name;
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

std::map<std::string, Skill>
ExecutionTimeEstimator::get_skills_from_config(const std::string &path) const
{
	const size_t                                  id_index       = 0;
	const size_t                                  property_index = 1;
	std::unique_ptr<Configuration::ValueIterator> it(config_->search(path.c_str()));
	std::map<std::string, std::string>            skill_strings;
	while (it->next()) {
		std::vector<std::string> skill_property =
		  str_split(std::string(it->path()).substr(path.size()));
		if (skill_property.size() != 2) {
			continue;
		}
		if (skill_property[property_index] == "args") {
			skill_strings[skill_property[id_index]] += str_join(it->get_strings(), ',');
		} else if (skill_property[property_index] == "name") {
			skill_strings[skill_property[id_index]] =
			  it->get_string() + "{" + skill_strings[skill_property[id_index]];
		}
	}
	std::map<std::string, Skill> res;
	for (const auto &skill_string : skill_strings) {
		res.insert(std::make_pair(skill_string.first, Skill(skill_string.second + "}")));
	}
	return res;
}

template <typename T>
T
ExecutionTimeEstimator::get_property(const Property<T> &property) const
{
	if (active_whitelist_entry_ == whitelist_.end()) {
		return property.get_default_value();
		;
	} else {
		return property.get_property(active_whitelist_entry_->first);
	}
}

template <typename T>
ExecutionTimeEstimator::Property<T>::Property(fawkes::Configuration  *config,
                                              const std::string      &path,
                                              const std::string      &property,
                                              const std::optional<T> &default_val)
{
	try {
		if constexpr (std::is_same<T, std::string>()) {
			default_value = config->get_string((path + property).c_str());
		} else if constexpr (std::is_same<T, float>()) {
			default_value = config->get_float((path + property).c_str());
		} else if constexpr (std::is_same<T, bool>()) {
			default_value = config->get_bool((path + property).c_str());
		} else {
			static_assert(dependent_false<T>::value,
			              "Property with this template type is not implemented");
		}
	} catch (Exception &e) {
		default_value = default_val;
	}
	const size_t                                  id_index       = 0;
	const size_t                                  property_index = 1;
	std::string                                   whitelist_path = path + "whitelist";
	std::unique_ptr<Configuration::ValueIterator> it(config->search(whitelist_path.c_str()));
	while (it->next()) {
		std::vector<std::string> skill_property =
		  str_split(std::string(it->path()).substr(whitelist_path.size()));
		if (skill_property.size() != 2) {
			break;
		}
		if (skill_property[property_index] == property) {
			if constexpr (std::is_same<T, std::string>()) {
				property_entries[skill_property[id_index]] += it->get_string();
			} else if constexpr (std::is_same<T, float>()) {
				property_entries[skill_property[id_index]] += it->get_float();
			} else if constexpr (std::is_same<T, bool>()) {
				property_entries[skill_property[id_index]] += it->get_bool();
			} else {
				static_assert(dependent_false<T>::value,
				              "Property with this template type is not implemented");
			}
		}
	}
}

template <typename T>
T
ExecutionTimeEstimator::Property<T>::get_default_value() const
{
	if (default_value) {
		return default_value.value();
	} else {
		throw Exception("failed to get property");
	}
}

template <typename T>
T
ExecutionTimeEstimator::Property<T>::get_property(const std::string &key) const
{
	auto property_specified = property_entries.find(key);
	if (property_specified == property_entries.end()) {
		return get_default_value();
	}
	return property_specified->second;
}

template class ExecutionTimeEstimator::Property<std::string>;
template class ExecutionTimeEstimator::Property<bool>;
template class ExecutionTimeEstimator::Property<float>;
template std::string
               ExecutionTimeEstimator::get_property(const Property<std::string> &property) const;
template bool  ExecutionTimeEstimator::get_property(const Property<bool> &property) const;
template float ExecutionTimeEstimator::get_property(const Property<float> &property) const;
} // namespace fawkes
