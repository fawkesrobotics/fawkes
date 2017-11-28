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

#include <fstream>
#include <streambuf>

#include "pddl_parser.h"

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

/** Parse the PDDL domain.
 * @param pddl_domain The PDDL domain as string (not a path)
 * @return A Domain object that contains the parsed domain.
 * @see Domain
 */
Domain
PddlParser::parseDomain(const std::string pddl_domain)
{
  typedef std::string::const_iterator iterator_type;
  typedef pddl_parser::grammar::domain_parser<iterator_type> grammar;
  typedef pddl_parser::grammar::pddl_skipper<iterator_type> skipper;

  grammar g;
  skipper s;
  Domain dom;

  std::string::const_iterator iter = pddl_domain.begin();
  std::string::const_iterator end = pddl_domain.end();
  bool r = phrase_parse(iter, end, g, s, dom);

  if ( !r ) {
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
PddlParser::parseProblem(const std::string pddl_problem)
{

  typedef std::string::const_iterator iterator_type;
  typedef pddl_parser::grammar::problem_parser<iterator_type> grammar;
  typedef pddl_parser::grammar::pddl_skipper<iterator_type> skipper;

  grammar g;
  skipper s;
  Problem prob;

  std::string::const_iterator iter = pddl_problem.begin();
  std::string::const_iterator end = pddl_problem.end();
  bool r = phrase_parse(iter, end, g, s, prob);

  if ( !r ) {
    throw PddlParserException("Parsing PDDL problem string failed!");
  }

  return prob;
}

}
