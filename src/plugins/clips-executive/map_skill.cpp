/***************************************************************************
 *  map_skill.cpp - Skill mapping  function
 *
 *  Created: Tue Sep 26 16:16:14 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
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

#include "map_skill.h"

#include <list>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#define REGEX_PARAM "\\?\\(([a-zA-Z0-9_-]+)((\\|/([^/]+)/([^/]+)/)*)\\)(s|S|i|f|y|Y)"

/** @class ActionSkillMapping "map_skill.h"
 * Class to maintain and perform mapping from actions to skills.
 * Given an action name and parameters, transform to skill string according to
 * some predetermined mapping.
 *
 * 
 * A mapping is a tuple of two elements:
 *  - parameter key or path element (left of the colon)
 *  - parameter value
 * These elements are described in the following.
 *
 * The configuration key or path element is the PDDL operator name.
 *
 * The mapping value can use the following elements as a pattern
 * modification for the skill string.
 * Note: parameters are always converted to lower-case by ROSPlan (at least
 *       in the default combination with POPF).
 *
 * Variable substitution has the general pattern ?(varname)M, where varname
 * is the name of the operator parameter and M a modifier. The modifier must
 * be one of:
 *  - s or S: convert value to string, that is add qutotation marks around
 *            the value. s passes the string as is, S converts to uppercase.
 *  - y or Y: pass value as symbol, i.e., the string value as is without
 *            quotation marks. Y additionally converts to upper case.
 *  - i:      converts the value to an integer
 *  - f:      converts the value to a float value
 *
 * Additionally, the arguments may be modified with a chain of regular
 * expressions. Then, the expression looks like this:
 * ?(var|/pattern/replace/|...)
 * There can be an arbitrary number of regular expressions chained by the
 * pipe (|) symbol. The "/pattern/replace/" can be a regular expression
 * according to the C++ ECMAScript syntax.
 * NOTE: the expressions may contain neither a slash (/) nor a pipe
 * (|), not even if quoted with a backslash. This is because a rather
 * simple pattern is used to extract the regex from the argument string.
 * The replacement string may contain reference to matched groups
 * (cf. http://ecma-international.org/ecma-262/5.1/ *sec-15.5.4.11). In
 * particular, the following might be useful:
 *  - $$: an actual dollar sign
 *  - $&: the full matched substring
 *  - $n: the n'th capture (may also be $nn for 10 <= nn <= 99)
 * Note that regular expression matching is performed case-insensitive, that
 * is because PDDL itself is also case-insensitive.
 * 
 *
 * == Examples ==
 * Examples contain three elements, the typed PDDL operator name with
 * parameters, the conversion string, and one or more conversion examples
 * of grounded actions to actuall skill strings.
 *
 * PDDL:  (move ?r - robot ?from ?to - location)
 * Param: move: ppgoto{place=?(to)S}
 * Examples: (move R-1 START C-BS-I) -> ppgoto{place="C-BS-I"}
 *
 * PDDL: (enter-field ?r - robot ?team-color - team-color)
 * Param: enter-field: drive_into_field{team=?(team-color)S, wait=?(r|/R-1/0.0/|/R-2/3.0/|/R-3/6.0/)f}
 * Examples: (enter_field R-1 CYAN) -> drive_into_field{team="CYAN", wait=0.000000}
 *           (enter_field R-2 CYAN) -> drive_into_field{team="CYAN", wait=3.000000}
 * Note: the chaining of regular expressions to fill in a parameter based on
 * the specific value of another parameter. You can also see that arguments
 * can be referenced more than once.
 *
 * @author Tim Niemueller
 */


/** Constructor. */
ActionSkillMapping::ActionSkillMapping()
{
}

/** Constructor with initial mapping.
 * @param mappings initial mapping
 */
ActionSkillMapping::ActionSkillMapping(std::map<std::string, std::string> &mappings)
	: mappings_(mappings)
{
}


/** Add another mapping.
 * @param action_name name of action to map
 * @param skill_string_template substitutation template
 */
void
ActionSkillMapping::add_mapping(std::string action_name, std::string skill_string_template)
{
	mappings_[action_name] = skill_string_template;
}

/** Check if mapping for an action exists.
 * @param action_name name of action to check
 * @return true if mapping exists, false otherwise
 */
bool
ActionSkillMapping::has_mapping(std::string &action_name) const
{
	return (mappings_.find(action_name) != mappings_.end());
}

/** Perform mapping
 * @param name name of action
 * @param params parameters as key value pairs
 * @param messages contains informational and error messages upon return.
 * The key denotes the severity, e.g., WARN or ERROR, the value is the actual
 * message.
 * @return The skill string of the mapped action, or an empty string in case of an error.
 */
std::string
ActionSkillMapping::map_skill(const std::string &name,
                              const std::map<std::string, std::string> &params,
                              std::multimap<std::string, std::string> &messages) const
{
	std::string rv;

	auto mapping = mappings_.find(name);
	if (mapping == mappings_.end())  return "";
	std::string remainder = mapping->second;

	std::regex re(REGEX_PARAM);
	std::smatch m;
	while (std::regex_search(remainder, m, re)) {
		bool found = false;
		for (const auto &p : params) {
			std::string value = p.second;
			if (p.first == m[1].str()) {
				found = true;
				rv += m.prefix();

				if (! m[2].str().empty()) {
					std::string rstr = m[2].str();
					std::list<std::string> rlst;
					std::string::size_type rpos = 0, fpos = 0;
					while ((fpos = rstr.find('|', rpos)) != std::string::npos) {
						std::string substr = rstr.substr(rpos, fpos-rpos);
						if (! substr.empty())  rlst.push_back(substr);
						rpos = fpos + 1;
					}
					rstr = rstr.substr(rpos);
					if (! rstr.empty())  rlst.push_back(rstr);
						
					for (const auto &r : rlst) {
						if (r.size() > 2 && r[0] == '/' && r[r.size()-1] == '/') {
							std::string::size_type slash_pos = r.find('/', 1);
							if (slash_pos != std::string::npos && slash_pos < (r.size() - 1)) {
								std::string r_match = r.substr(1, slash_pos - 1);
								std::string r_repl  = r.substr(slash_pos + 1, (r.size() - slash_pos - 2));
								std::regex user_regex(r_match, std::regex::ECMAScript|std::regex::icase);
								value = std::regex_replace(value, user_regex, r_repl);
							} else {
								messages.insert(std::make_pair("WARN", " regex '" + r + "' missing mid slash, ignoring"));
							}
						} else {
							messages.insert(std::make_pair("WARN", "regex '" + r + "' missing start/end slashes, ignoring"));
						}
					}
				}

				switch (m[6].str()[0]) {
				case 's': rv += "\"" + value + "\""; break;
				case 'S':
					{
						std::string uc = value;
						std::transform(uc.begin(), uc.end(), uc.begin(), ::toupper);
						rv += "\"" + uc + "\"";
					}
					break;
				case 'y': rv += value; break;
				case 'Y':
					{
						std::string uc = value;
						std::transform(uc.begin(), uc.end(), uc.begin(), ::toupper);
						rv += uc;
					}
					break;
				case 'i':
					try {
						rv += std::to_string(std::stol(value));
					} catch (std::invalid_argument &e) {
						messages.insert(std::make_pair("ERROR", "Failed to convert '" + value + "' to integer: " + e.what()));
						return "";
					}
					break;
							
				case 'f':
					try {
						rv += std::to_string(std::stod(value));
					} catch (std::invalid_argument &e) {
						messages.insert(std::make_pair("ERROR", "Failed to convert '" + value + "' to float: " + e.what()));
						return "";
					}
					break;
				}
				break;
			}
		}
		if (! found) {
			messages.insert(std::make_pair("ERROR", "No value for parameter '" +
			                               m[1].str() + "' of action '" + name + "' given"));
			return "";
		}

		remainder = m.suffix();
	}
	rv += remainder;
		
	return rv;
}

}
