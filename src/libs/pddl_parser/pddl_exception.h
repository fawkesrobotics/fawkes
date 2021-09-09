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

/** @class PddlParserException <pddl_parser/pddl_exception.h>
 * Exception thrown by the parser if an error occurs during parsing.
 */

class PddlParserException : public fawkes::Exception
{
public:
	/** Constructor.
    * @param msg A message describing the error.
    */
	PddlParserException(const char *msg) : fawkes::Exception(msg)
	{
	}

	const char *
	what() const throw() override
	{
		if (messages != NULL) {
			return messages->msg;
		} else {
			return "Unknown Error";
		}
	}
	/** Constructor with a string message.
   * This wraps the constructor with a char* message for usage with std::string.
   * @param msg A message describing the error.
   */
	PddlParserException(const std::string &msg) : fawkes::Exception(msg.c_str())
	{
	}
};

/** @class PddlSemanticsException <pddl_parser/pddl_exception.h>
 * Exception thrown by the parser if an error occurs during semantic checks
 * during parsing.
 */
class PddlSemanticsException : public PddlParserException
{
public:
	/** Position of the error to generate a helpful error message. */
	const iterator_type pos;
	/** Constructor.
    * @param msg A message describing the error.
    * @param pos The position in the parsed string where the error occurs.
    */
	PddlSemanticsException(const char *msg, const iterator_type &pos)
	: PddlParserException(msg), pos(pos)
	{
	}
	/** Constructor with a string message.
   * This wraps the constructor with a char* message for usage with std::string.
   * @param msg A message describing the error.
   * @param pos The position in the parsed string where the error occurs.
   */
	PddlSemanticsException(const std::string &msg, const iterator_type &pos)
	: PddlParserException(msg.c_str()), pos(pos)
	{
	}
};

/** @class PddlTypeException <pddl_parser/pddl_exception.h>
 * Exception thrown by the parser if declared type does not match the defined
 * one.
 */
class PddlTypeException : public PddlSemanticsException
{
	using PddlSemanticsException::PddlSemanticsException;
};
/** @class PddlSyntaxException <pddl_parser/pddl_exception.h>
 * Exception thrown by the parser if there is a syntax error.
 */
class PddlSyntaxException : public PddlSemanticsException
{
	using PddlSemanticsException::PddlSemanticsException;
};
/** @class PddlPredicateException <pddl_parser/pddl_exception.h>
 * Exception thrown by the parser if a declared relation does not match the
 * defined predicate.
 */
class PddlPredicateException : public PddlSemanticsException
{
	using PddlSemanticsException::PddlSemanticsException;
};
/** @class PddlExpressionException <pddl_parser/pddl_exception.h>
 * Exception thrown by the parser if an expression is invalid.
 */
class PddlExpressionException : public PddlSemanticsException
{
	using PddlSemanticsException::PddlSemanticsException;
};
/** @class PddlConstantException <pddl_parser/pddl_exception.h>
 * Exception thrown by the parser if a declared constant does not match a
 * defined one.
 */
class PddlConstantException : public PddlSemanticsException
{
	using PddlSemanticsException::PddlSemanticsException;
};
/** @class PddlParameterException <pddl_parser/pddl_exception.h>
 * Exception thrown by the parser if a parameter mismatch is encountered.
 */
class PddlParameterException : public PddlSemanticsException
{
	using PddlSemanticsException::PddlSemanticsException;
};

} // end namespace pddl_parser
#endif /* !PDDL_EXCEPTION_H */
