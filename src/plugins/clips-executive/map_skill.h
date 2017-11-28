/***************************************************************************
 *  map_skill.h - Skill mapping  function
 *
 *  Created: Tue Sep 26 16:15:00 2017
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

#include <map>
#include <string>
#include <regex>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ActionSkillMapping
{
 public:
	ActionSkillMapping();
	ActionSkillMapping(std::map<std::string, std::string> &mappings);

	void add_mapping(std::string action_name, std::string skill_string_template);
	bool has_mapping(std::string &action_name) const;

	std::string
		map_skill(const std::string &name, const std::map<std::string, std::string> &params,
		          std::multimap<std::string, std::string> &messages) const;

 private:
	std::map<std::string, std::string> mappings_;
};

}
