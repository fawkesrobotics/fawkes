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
	std::type_index
	operator()(const QuantifiedFormula &p) const
	{
		return std::type_index(typeid(p));
	}
};

struct TypeSemantics
{
	pair_type
	operator()(const iterator_type &where, const pair_type &parsed, const Domain &domain) const;
};

struct ParamTransformer
{
	pair_type operator()(const iterator_type &    where,
	                     const pair_strings_type &parsed,
	                     string_pairs_type &      target) const;
};

struct ConstantSemantics
{
	pair_multi_const operator()(const iterator_type &     where,
	                            const pair_multi_const &  parsed,
	                            const Domain &            domain,
	                            std::vector<std::string> &warnings) const;
};

struct ActionSemantics
{
	static bool check_type(const iterator_type &where,
	                       const std::string &  got,
	                       const std::string &  expected,
	                       const Domain &       domain);

	static void check_action_condition(const iterator_type &where,
	                                   const Expression &   expr,
	                                   const Domain &       domain,
	                                   const Action &       action,
	                                   string_pairs_type &  bound_vars);
	static void check_action_predicate(const iterator_type & where,
	                                   const Predicate &     pred,
	                                   const ExpressionType &type,
	                                   const Domain &        domain,
	                                   const Action &        action,
	                                   string_pairs_type &   bound_vars);

	Action operator()(const iterator_type &where, const Action &parsed, const Domain &domain) const;
};

} // namespace pddl_parser

#endif /* PLUGINS_PDDL_SEMANTICS_H */
