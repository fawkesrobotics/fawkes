/***************************************************************************
 *  pddl_semantics.h
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

#ifndef PLUGINS_PDDL_SEMANTICS_H
#define PLUGINS_PDDL_SEMANTICS_H

#include "pddl_ast.h"

#include <core/exception.h>

#include <typeindex>

namespace pddl_parser {
struct ExpressionTypeVisitor : public boost::static_visitor<std::type_index>
{
	std::type_index
	operator()(const Atom &a) const
	{
		return std::type_index(typeid(a));
	}
	std::type_index
	operator()(const Predicate &p) const
	{
		return std::type_index(typeid(p));
	}
};

class PddlSemanticsException : public fawkes::Exception
{
public:
	iterator_type pos;
	/** Constructor.
    * @param msg A message describing the error.
    */
	PddlSemanticsException(const char *msg, const iterator_type &pos)
	: fawkes::Exception(msg), pos(pos)
	{
	}
	/** Constructor with a string message.
   * This wraps the constructor with a char* message for usage with std::string.
   * @param msg A message describing the error.
   */
	PddlSemanticsException(const std::string &msg, const iterator_type &pos)
	: fawkes::Exception(msg.c_str()), pos(pos)
	{
	}
};

struct ActionSemantics
{
	static bool check_type(const iterator_type &where,
	                       const std::string &  got,
	                       const std::string &  expected,
	                       const Domain &       domain);

	static bool check_action_predicates(const iterator_type &where,
	                                    const Expression &   expr,
	                                    const Domain &       domain,
	                                    const Action &       action);

	Action operator()(const iterator_type &where, const Action &parsed, const Domain &domain) const;
};

} // namespace pddl_parser

#endif /* PLUGINS_PDDL_SEMANTICS_H */
