/***************************************************************************
 *  stn.cpp - stn-generator
 *
 *  Created: Sun Dec 22 22:41 2019
 *  Copyright  Nils Adermann <naderman@naderman.de>, Daniel Habering
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

#ifndef PDDLQI_PARSER_PARSER_H
#define BOOST_SPIRIT_DEBUG
#define PDDLQI_PARSER_PARSER_H

#include "pddl_grammar.h"

#include <iostream>

namespace pddl_parser {
class Parser
{
public:
	PddlDomain
	parseDomain(const std::string &input)
	{
		return parse<Grammar::Domain<std::string::const_iterator>,
		             Grammar::pddl_skipper<std::string::const_iterator>,
		             PddlDomain>(input);
	}

	PddlProblem
	parseProblem(const std::string &input)
	{
		return parse<Grammar::Problem<std::string::const_iterator>,
		             Grammar::pddl_skipper<std::string::const_iterator>,
		             PddlProblem>(input);
	}

	template <typename Grammar, typename Skipper, typename Attribute>
	Attribute
	parse(const std::string &input)
	{
		Skipper skipper;

		Grammar grammar;

		Attribute data;

		std::string::const_iterator iter = input.begin();
		std::string::const_iterator end  = input.end();

		bool r = phrase_parse(iter, end, grammar, skipper, data);

		if (!r || iter != end) {
			//std::cout << "Failed randomly?" << std::endl;
			throw ParserException(input.begin(), iter, end);
		}

		return data;
	}
};
} // namespace pddl_parser
#endif
