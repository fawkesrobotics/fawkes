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

/** @class ExpressionTypeVisitor
 * Retrieve the type index of an expression_t expression to determine the
 * underlying type of the variant.
 */
struct ExpressionTypeVisitor : public boost::static_visitor<std::type_index>
{
	/** Visitor for Atom.
	 * @param a Atom.
	 * @return Type index of struct Atom.
	 */
	std::type_index
	operator()(const Atom &a) const
	{
		return std::type_index(typeid(a));
	}
	/** Visitor for Predicate.
	 * @param p Predicate.
	 * @return Type index of struct Predicate.
	 */
	std::type_index
	operator()(const Predicate &p) const
	{
		return std::type_index(typeid(p));
	}
	/** Visitor for QuantifiedFormula.
	 * @param p Quantified formula.
	 * @return Type index of struct QuantifiedFormula.
	 */
	std::type_index
	operator()(const QuantifiedFormula &p) const
	{
		return std::type_index(typeid(p));
	}
};

/** @class TypeSemantics
 * Functor for semantic checks when parsing PDDL types.
 */
struct TypeSemantics
{
	/**
	 * Throw an exception if the parsed type is a sub-type but the domain does not
	 * have the requirement :typing enabled.
	 * @param where Position of the parsed type in the string to parse.
	 * @param parsed Type that got parsed.
	 * @param domain Partial domain containing everything that was parsed so far
	 * @return the parsed type.
	 */
	pair_type
	operator()(const iterator_type &where, const pair_type &parsed, const Domain &domain) const;
};

/** @class ParamTransformer
 * Functor to uniformly handle disjunctive types and shorthand notations.
 */
struct ParamTransformer
{
	/**
	 * Transform a pair of string vectors to pairs of strings.
	 *
	 * Parameters may be given in a form '?a ?b - (either x y)', which is parsed
	 * as <[a, b],[x,y]>. The Transformation creates <a,x> <b,x>, <a,y> and
	 * <b,y> out of this.
	 *
	 * @param where Position of the parsed param type in the string to parse.
	 * @param parsed Parameters that got parsed.
	 * @param target The vector that is extended by all the constructed pairs.
	 * @return The last created param tuple, this is not added to target in this
	 *         function but rather is added through the semantic action when
	 *         parsing.
	 */
	pair_type operator()(const iterator_type &    where,
	                     const pair_strings_type &parsed,
	                     string_pairs_type &      target) const;
};

/** @class ConstantSemantics
 * Functor for semantic checks when parsing constants of a PDDL domain.
 */
struct ConstantSemantics
{
	/**
	 * Check whether the given type for a set of constants is defined and
	 * registers warnings if constants are defined multiple times with conflicting
	 * types.
	 *
	 * @param where Position of the parsed constants.
	 * @param parsed Constants that got parsed.
	 * @param domain Partial domain containing everything that was parsed so far
	 * @param warnings Hook from the parser where non-fatal warnings can be stored
	 * @return the parsed constants.
	 */
	pair_multi_const operator()(const iterator_type &     where,
	                            const pair_multi_const &  parsed,
	                            const Domain &            domain,
	                            std::vector<std::string> &warnings) const;
};

/** @class ActionSemantics
 * Functor for semantic checks when parsing actions of a PDDL domain.
 */
struct ActionSemantics
{
	/**
	 * Helper to check whether a type matches the expected one.
	 *
	 * Recursively steps up the type hierarchy until the expected type is found or
	 * the topmost level is reached.
	 *
	 * @param where Position of the parsed action.
	 * @param got Type that has to be checked.
	 * @param expected Type that is to be matched.
	 * @param domain partial domain containing everything that was parsed so far
	 * @return true iff got is a sub-type of/the same type as expected.
	 */
	static bool check_type(const iterator_type &where,
	                       const std::string &  got,
	                       const std::string &  expected,
	                       const Domain &       domain);

	/**
	 * Helper to recursively check expression semantics within precondition and
	 * effects of actions.
	 *
	 * @param where Position of the parsed action.
	 * @param expr Expression to be checked.
	 * @param domain Partial domain containing everything that was parsed so far.
	 * @param action Action containing the expression to check.
	 * @param bound_vars Variables that are bound through quantified formulas on
	 * 				an upper recursion level.
	 * */
	static void check_action_condition(const iterator_type &where,
	                                   const Expression &   expr,
	                                   const Domain &       domain,
	                                   const Action &       action,
	                                   string_pairs_type &  bound_vars);
	/**
	 * Helper to recursively check expression semantics within predicates.
	 *
	 * @param where Position of the parsed action.
	 * @param pred Predicate to be checked.
	 * @param type Expression type of the predicate.
	 * @param domain Partial domain containing everything that was parsed so far.
	 * @param action Action containing the expression to check.
	 * @param bound_vars Variables that are bound through quantified formulas on
	 * 				an upper recursion level.
	 * */
	static void check_action_predicate(const iterator_type & where,
	                                   const Predicate &     pred,
	                                   const ExpressionType &type,
	                                   const Domain &        domain,
	                                   const Action &        action,
	                                   string_pairs_type &   bound_vars);

	/**
	 * Check whether the parameter list is properly typed and all expressions
	 * that express conditions/effects are well-formed in the domain.
	 *
	 * @param where Position of the parsed action.
	 * @param parsed Action that got parsed.
	 * @param domain Partial domain containing everything that was parsed so far.
	 * @return parsed Action.
	 */
	Action operator()(const iterator_type &where, const Action &parsed, const Domain &domain) const;
};

} // namespace pddl_parser

#endif /* PLUGINS_PDDL_SEMANTICS_H */
