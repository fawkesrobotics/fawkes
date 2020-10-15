
/***************************************************************************     
 *  pddl_parser.h
 *     
 *  Created: Fri 19 May 2017 11:10:30 CEST
 *  Copyright  2017  Matthias Loebach     
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

#ifndef PLUGINS_PDDL_PARSER_H_
#define PLUGINS_PDDL_PARSER_H_

#include "pddl_exception.h"
#include "pddl_grammar.h"

#include <boost/spirit/include/support_line_pos_iterator.hpp>
#include <string>

namespace pddl_parser {

class PddlParser
{
public:
	static Domain  parseDomain(const std::string &pddl_domain);
	static Problem parseProblem(const std::string &pddl_problem);

private:
	static std::string
	getErrorContext(const iterator_type &, const iterator_type &, const iterator_type &);
};

} // namespace pddl_parser

#endif
