/***************************************************************************
 *  effect_visitor.cpp - A static visitor to translate an effect
 *
 *  Created: Tue 31 Oct 2017 12:39:11 CET 12:39
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "effect_visitor.h"

#include "core/exception.h"

#include <pddl_parser/pddl_exception.h>

/** @class EffectToCLIPSFactVisitor "effect_visitor.h"
 * Translate a PDDL effect into CLIPS facts.
 * @author Till Hofmann
 * Helper class to translate an effect from pddl_parser::Expression to a CLIPS
 * fact.  An expression is a boost::variant, and this class is a visitor for the
 * variant that translates the Expression into a a vector of CLIPS facts.
 */

/** Constructor.
 * @param pddl_operator The name of the operator this effect belongs to.
 * @param positive True iff this is a positive (not a negative) effect.
 */
EffectToCLIPSFactVisitor::EffectToCLIPSFactVisitor(const std::string &pddl_operator, bool positive)
: pddl_operator_(pddl_operator), positive_effect_(positive)
{
}

/** Translate a quantified formula to a vector of strings.
 * Not implemented yet.
 * @param q The quantified formula to translate into a string.
 * @return An empty vector.
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(pddl_parser::QuantifiedFormula &q) const
{
	throw pddl_parser::PddlParserException("QuantifiedFormulas are not supported in CLIPS yet.");
	return std::vector<std::string>();
}

/** Translate an Atom into a vector of strings.
 * Note that this does not return a CLIPS fact because we do not store atoms
 * (parameter names or constants) as separate facts. This needs to be further
 * processed by the caller instead.
 * @param a The atom to translate into a string.
 * @return A vector that only contains the atom as is.
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(pddl_parser::Atom &a) const
{
	return std::vector<std::string>({a});
}

/** Translate a Predicate into a vector of strings.
 * This creates proper CLIPS effect fact strings for the Predicate and all its
 * arguments. For compound formulae (e.g., conjunctions), this also translates
 * all sub-formulae recursively.
 * @param p The predicate to translate.
 * @return A vector of strings, each string is a properly formed CLIPS fact.
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(pddl_parser::Predicate &p) const
{
	std::vector<std::string> res;
	if (p.function == "and") {
		for (pddl_parser::Expression &sub : p.arguments) {
			std::vector<std::string> sub_effects =
			  boost::apply_visitor(EffectToCLIPSFactVisitor(pddl_operator_, positive_effect_),
			                       sub.expression);
			res.insert(res.end(), sub_effects.begin(), sub_effects.end());
		}
	} else if (p.function == "not") {
		if (p.arguments.size() != 1) {
			throw pddl_parser::PddlParserException("Expected exactly one sub-formula for 'not'");
		}
		std::vector<std::string> sub_effects =
		  boost::apply_visitor(EffectToCLIPSFactVisitor(pddl_operator_, !positive_effect_),
		                       p.arguments[0].expression);
		res.insert(res.end(), sub_effects.begin(), sub_effects.end());
	} else if (p.function == "at start" || p.function == "over all" || p.function == "at end") {
		// This is a temporal effect. We ignore the temporal aspect and for now just use the sub-expression.
		if (p.arguments.size() != 1) {
			throw fawkes::Exception(
			  "Unexpected number of sub-formulas (%zu) of temporal formula, expected exactly 1",
			  p.arguments.size());
		}

		auto sub_expr = boost::apply_visitor(EffectToCLIPSFactVisitor(pddl_operator_, positive_effect_),
		                                     p.arguments[0].expression);
		res.insert(res.end(), sub_expr.begin(), sub_expr.end());
		return res;
	} else {
		// We expect p.function to be a predicate name.
		std::string params    = "";
		std::string constants = "";
		for (auto &arg : p.arguments) {
			std::vector<std::string> p_strings =
			  boost::apply_visitor(EffectToCLIPSFactVisitor(pddl_operator_, positive_effect_),
			                       arg.expression);
			if (p.function.find(" ") != std::string::npos) {
				throw fawkes::Exception("Parser error: Expected '%s' to be a predicate name, but it "
				                        "contains a space. Is it really a domain predicate?",
				                        p.function.c_str());
			}
			if (p_strings.size() != 1) {
				throw fawkes::Exception(
				  "Parser error: Unexpected parameter length (%zu) while parsing predicate parameter in "
				  "effect of %s, expected exactly one paramter. Is '%s' really a domain predicate?",
				  p_strings.size(),
				  pddl_operator_.c_str(),
				  p.function.c_str());
			}
			std::string p_string = p_strings[0];
			if (p_string[0] == '?') {
				// It's really a parameter.
				if (p_string.length() <= 1) {
					throw pddl_parser::PddlParserException("Invalid parameter name " + p_string);
				}
				params += " " + p_string.substr(1);
				constants += " nil";
			} else {
				// It's a constant.
				params += " c";
				constants += " " + p_string;
			}
		}
		res.push_back(std::string("(domain-effect"
		                          " (part-of "
		                          + pddl_operator_
		                          + ")"
		                            " (predicate "
		                          + p.function
		                          + ")"
		                            " (param-names "
		                          + params
		                          + ")"
		                            " (param-constants "
		                          + constants
		                          + ")"
		                            " (type "
		                          + (positive_effect_ ? "POSITIVE" : "NEGATIVE")
		                          + ")"
		                            ")"));
	}
	return res;
}
