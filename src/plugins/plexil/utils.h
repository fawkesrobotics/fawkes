
/***************************************************************************
 *  utils.h -  PLEXIL utils
 *
 *  Created: Fri Aug 17 16:15:54 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PLEXIL_UTILS_H_
#define __PLUGINS_PLEXIL_UTILS_H_

#include <Error.hh>
#include <Debug.hh>
#include <Value.hh>

#include <string>
#include <map>
#include <vector>

inline void
replace_tokens(std::string &s)
{
	std::map<std::string, std::string> tokens =
	  {{"@CFGDIR@", CONFDIR},
	   {"@BASEDIR@", BASEDIR},
	   {"@FAWKES_BASEDIR@", FAWKES_BASEDIR},
	   {"@RESDIR@", RESDIR}};

	for (const auto &token : tokens) {
		std::string::size_type pos;
		if ((pos = s.find(token.first)) != std::string::npos) {
			s.replace(pos, token.first.size(), token.second);
		}
	}
}

inline bool
verify_args(const std::vector<PLEXIL::Value> &args, const std::string& func,
            const std::vector<std::pair<std::string, PLEXIL::ValueType>> &types)
{
	if (args.size() != types.size()) {
		warn(func << ": Command requires " << types.size() << " arguments, got " << args.size());
		for (size_t i = 0; i < args.size(); ++i) {
			warn(func << ": Argument " << i << " = " << args[i]);
		}
		return false;
	}
	for (size_t i = 0; i < args.size(); ++i) {
		// Treat UNKNOWN_TYPE as "any type and we don't care/inspect later"
		if (types[i].second == PLEXIL::UNKNOWN_TYPE) continue;

		if (args[i].valueType() != types[i].second) {
			warn(func << ":"
			     << "Command argument " << i << "(" << types[i].first << ") expected to be of type "
			     << PLEXIL::valueTypeName(types[i].second) << ", but is of type "
			     << PLEXIL::valueTypeName(args[i].valueType()));
			return false;
		}
	}
	return true;
}


#endif
