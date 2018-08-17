
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

#include <string>

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

#endif
