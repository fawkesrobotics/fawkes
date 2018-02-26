    
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

#ifndef __PLUGINS_PDDL_PARSER_H
#define __PLUGINS_PDDL_PARSER_H

#include "pddl_grammar.h"

#include <core/exception.h>

#include <string>

namespace pddl_parser {

class PddlParser
{
 public:
  static Domain parseDomain(const std::string pddl_domain);
  static Problem parseProblem(const std::string pddl_problem);
 private:
};

/** @class PddlParserException <pddl_parser/pddl_parser.h>
 * Exception thrown by the parser if an error occurs during parsing.
 */

class PddlParserException : public fawkes::Exception
{
 public:
   /** Constructor.
    * @param msg A message describing the error.
    */
  PddlParserException(const char *msg)
    : fawkes::Exception(msg) {}
  /** Constructor with a string message.
   * This wraps the constructor with a char* message for usage with std::string.
   * @param msg A message describing the error.
   */
  PddlParserException(const std::string &msg)
    : fawkes::Exception(msg.c_str()) {}
};

}

#endif
