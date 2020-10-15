/***************************************************************************
 *  pddl_exception.cpp exceptions thrown durning parsing
 *
 *  Created: Thursday 15 October 2020
 *  Copyright  2020  Tarik Viehmann
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

#ifndef PDDL_EXCEPTION_H
#define PDDL_EXCEPTION_H

#include "pddl_ast.h"

#include <core/exception.h>

#include <string>

namespace pddl_parser {
enum PddlErrorType {
	TYPE_ERROR,
	SYNTAX_ERROR,
	PREDICATE_ERROR,
	EXPRESSION_ERROR,
	CONSTANT_ERROR,
	PARAMETER_ERROR,
	UNKNOWN_ERROR
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
	PddlParserException(const char *         msg,
	                    const PddlErrorType &error_type = PddlErrorType::UNKNOWN_ERROR)
	: fawkes::Exception(msg), error_type(error_type)
	{
	}
	/** Constructor with a string message.
   * This wraps the constructor with a char* message for usage with std::string.
   * @param msg A message describing the error.
   */
	PddlParserException(const std::string &  msg,
	                    const PddlErrorType &error_type = PddlErrorType::UNKNOWN_ERROR)
	: fawkes::Exception(msg.c_str()), error_type(error_type)
	{
	}

	const PddlErrorType error_type;
};

class PddlSemanticsException : public PddlParserException
{
public:
	const iterator_type pos;
	/** Constructor.
    * @param msg A message describing the error.
    */
	PddlSemanticsException(const char *msg, const PddlErrorType &error_type, const iterator_type &pos)
	: PddlParserException(msg, error_type), pos(pos)
	{
	}
	/** Constructor with a string message.
   * This wraps the constructor with a char* message for usage with std::string.
   * @param msg A message describing the error.
   */
	PddlSemanticsException(const std::string &  msg,
	                       const PddlErrorType &error_type,
	                       const iterator_type &pos)
	: PddlParserException(msg.c_str(), error_type), pos(pos)
	{
	}
};

} // end namespace pddl_parser
#endif /* !PDDL_EXCEPTION_H */
