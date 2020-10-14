/***************************************************************************     
 *  pddl_parser.cpp
 *     
 *  Created: Fri 19 May 2017 11:10:01 CEST
 *  Copyright  2017  Matthias Loebach     
 *                   Till Hofmann
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

#include "pddl_parser.h"

#include <fstream>
#include <streambuf>

namespace pddl_parser {

/** @class PddlParser <pddl_parser/pddl_parser.h>
 * Parse a PDDL domain file or problem.
 * This class parses a domain/problem into a structured representation of
 * the domain, which can then be used by other components.
 * @see pddl_ast.h
 */

/** @class PddlParserException <pddl_parser/pddl_parser.h>
 * Exception thrown by the parser if an error occurs during parsing.
 */

/** provide a description of the position in a file, where an error occurred.
 * @param start_it begin of parsing context
 * @param end_it end of parsing context
 * @param current_it position in the parsing context
 * \return a string containing the line at \a current_it and an indication
 * of the current position
 */
std::string
PddlParser::getErrorContext(const iterator_type &start_it,
                            const iterator_type &end_it,
                            const iterator_type &current_it)
{
	auto               line = get_current_line(start_it, current_it, end_it);
	std::ostringstream error_msg;
	error_msg << " line:" << get_line(current_it) << ", col:" << get_column(line.begin(), current_it)
	          << "\n";
	while (!line.empty() && std::strchr("\r\n", *line.begin()))
		line.advance_begin(1);
	error_msg << line << "\n";
	error_msg << std::string(std::distance(line.begin(), current_it), ' ')
	          << "^ --- last correctly parsed\n";
	return error_msg.str();
}

/** Parse the PDDL domain.
 * @param pddl_domain The PDDL domain as string (not a path)
 * @return A Domain object that contains the parsed domain.
 * @see Domain
 */
Domain
PddlParser::parseDomain(const std::string &pddl_domain)
{
	typedef pddl_parser::grammar::domain_parser<iterator_type> grammar;
	typedef pddl_parser::grammar::pddl_skipper<iterator_type>  skipper;

	skipper s;
	Domain  dom;
	bool    r = false;

	iterator_type iter(pddl_domain.begin());
	iterator_type end(pddl_domain.end());

	grammar g;

	try {
		r = phrase_parse(iter, end, g, s, dom);
	} catch (qi::expectation_failure<iterator_type> const &e) {
		throw PddlParserException(std::string("Expectation failed: ") + e.what()
		                          + getErrorContext(iter, end, e.first));
	}
	if (!r) {
		throw PddlParserException("Parsing PDDL domain string failed!");
	}

	return dom;
}

/** Parse the PDDL problem.
 * @param pddl_problem The problem as string (not a path)
 * @return A Problem object that contains the parsed problem.
 * @see Problem
 */
Problem
PddlParser::parseProblem(const std::string &pddl_problem)
{
	typedef pddl_parser::grammar::problem_parser<iterator_type> grammar;
	typedef pddl_parser::grammar::pddl_skipper<iterator_type>   skipper;

	grammar g;
	skipper s;
	Problem prob;
	bool    r = false;

	iterator_type iter(pddl_problem.begin());
	iterator_type end(pddl_problem.end());

	try {
		r = phrase_parse(iter, end, g, s, prob);
	} catch (qi::expectation_failure<iterator_type> const &e) {
		throw PddlParserException(std::string("Expectation failed: ") + e.what()
		                          + getErrorContext(iter, end, e.first));
	}

	if (!r) {
		throw PddlParserException("Parsing PDDL problem string failed!");
	}

	return prob;
}

} // namespace pddl_parser
