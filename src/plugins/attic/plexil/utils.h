
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

#include <Debug.hh>
#include <Error.hh>
#include <Value.hh>
#include <cstring>
#include <map>
#include <string>
#include <vector>

inline void
replace_tokens(std::string &s)
{
	std::map<std::string, std::string> tokens = {{"@CFGDIR@", CONFDIR},
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
verify_args(const std::vector<PLEXIL::Value>                             &args,
            const std::string                                            &func,
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
		if (types[i].second == PLEXIL::UNKNOWN_TYPE)
			continue;

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

/** Get configuration value from XML adapter config.
 * Value name can either reference an attribute name, or a <Parameter> tag with
 * the key attribte set to the given value. The attribute is preferred.
 * @param config configuration to query, typically retrieved as getXml()
 * @param name name/key of configuration value
 * @return value as string
 */
inline std::string
get_xml_config_value(const pugi::xml_node &config, const std::string &name)
{
	pugi::xml_attribute xml_attr = config.attribute(name.c_str());
	if (xml_attr) {
		return xml_attr.value();
	} else {
		for (const auto &c : config.children()) {
			if (strcmp(c.name(), "Parameter") == 0) {
				pugi::xml_attribute xml_key_attr = c.attribute("key");
				if (xml_key_attr && strcmp(xml_key_attr.value(), name.c_str()) == 0) {
					return c.text().get();
				}
			}
		}
	}

	return "";
}

#endif
