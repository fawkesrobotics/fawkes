/***************************************************************************
 *  precondition_visitor.cpp - A static visitor to translate a precondition
 *
 *  Created: Mon 16 Oct 2017 18:34:44 CEST 18:34
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

#include "precondition_visitor.h"

#include "core/exception.h"

#include <pddl_parser/pddl_exception.h>

/** @class PreconditionToCLIPSFactVisitor "precondition_visitor.h"
 * Translate a PDDL precondition into CLIPS facts.
 * @author Till Hofmann
 * Helper class to translate a precondition from pddl_parser::Expression to a
 * CLIPS fact.  An expression is a boost::variant, and this class is a visitor
 * for the variant that translates the Expression into a a vector of CLIPS
 * facts.
 */

/** Constructor.
 * @param parent The name of the parent (either an operator or a precondition)
 * @param sub_counter Counter passed by the parent to enumerate sub-conditions
 * @param is_main true if this is the direct child of the operator,
 * i.e., not a sub-condition
 */
PreconditionToCLIPSFactVisitor::PreconditionToCLIPSFactVisitor(const std::string &parent,
                                                               int                sub_counter,
                                                               bool is_main /* = false */)
: parent_(parent), sub_counter_(sub_counter), is_main_(is_main)
{
}

/** Translate a quantified formula to a vector of strings.
 * Not implemented yet.
 * @param q The quantified formula to translate into a string.
 * @return An empty vector.
 */
std::vector<std::string>
PreconditionToCLIPSFactVisitor::operator()(pddl_parser::QuantifiedFormula &q) const
{
	std::vector<std::string> res;
	std::stringstream        typestream;
	std::stringstream        namestream;

	std::stringstream identifierstream;
	identifierstream << parent_ << sub_counter_;
	std::string id = identifierstream.str();

	for (const auto &value : q.args) {
		typestream << value.second << " ";
		namestream << value.first << " ";
	}
	res.push_back(std::string("(pddl-formula "
	                          " (id "
	                          + id
	                          + ") "
	                            " (part-of "
	                          + parent_
	                          + ") "
	                            " (type "
	                          + q.quantifier
	                          + ") "
	                            "(quantified-names "
	                          + namestream.str()
	                          + ") "
	                            "(quantified-types "
	                          + typestream.str() + "))"

	                          ));

	std::vector<std::string> args =
	  boost::apply_visitor(PreconditionToCLIPSFactVisitor(id, 1), q.sub_expr.expression);
	res.insert(res.end(), args.begin(), args.end());

	return res;
}

/** Translate an Atom into a vector of strings.
 * Note that this does not return a CLIPS fact because we do not store atoms
 * (parameter names or constants) as separate facts. This needs to be further
 * processed by the caller instead.
 * @param a The atom to translate into a string.
 * @return A vector that only contains the atom as is.
 */
std::vector<std::string>
PreconditionToCLIPSFactVisitor::operator()(pddl_parser::Atom &a) const
{
	return {a};
}

/** Translate a Predicate into a vector of strings.
 * This creates proper CLIPS precondition fact strings for the Predicate and all
 * its arguments. For compound formulae (e.g., conjunctions), this also
 * translates all sub-formulae recursively.
 * @param p The predicate to translate.
 * @return A vector of strings, each string is a properly formed CLIPS fact.
 */
std::vector<std::string>
PreconditionToCLIPSFactVisitor::operator()(pddl_parser::Predicate &p) const
{
	std::vector<std::string> res;
	std::stringstream        namestream;
	namestream << parent_ << sub_counter_;
	std::string name = namestream.str();
	if (p.function == "and" || p.function == "not" || p.function == "or") {
		std::string type;
		if (p.function == "and") {
			type = "conjunction";
		} else if (p.function == "or") {
			type = "disjunction";
		} else if (p.function == "not") {
			type = "negation";
		}

		res.push_back(std::string("(pddl-formula"
		                          " (id "
		                          + name
		                          + ")"
		                            " (part-of "
		                          + parent_
		                          + ")"
		                            " (type "
		                          + type
		                          + ")"
		                            ")"));
		uint sub_counter = 1;
		for (pddl_parser::Expression &sub : p.arguments) {
			std::vector<std::string> args =
			  boost::apply_visitor(PreconditionToCLIPSFactVisitor(name, sub_counter++), sub.expression);
			res.insert(res.end(), args.begin(), args.end());
		}
		return res;
	} else if (p.function == "at start" || p.function == "over all" || p.function == "at end") {
		// This is a temporal condition. We ignore the temporal aspect for now and just use the sub-expression.
		if (p.arguments.size() != 1) {
			throw fawkes::Exception(
			  "Unexpected number of sub-formulas (%zu) of temporal formula, expected exactly 1",
			  p.arguments.size());
		}

		auto sub_expr = boost::apply_visitor(PreconditionToCLIPSFactVisitor(parent_, sub_counter_),
		                                     p.arguments[0].expression);
		res.insert(res.end(), sub_expr.begin(), sub_expr.end());
		return res;
	} else {
		// We expect p.function to be a predicate name.
		std::string new_parent;
		if (is_main_) {
			// Special case: this is the main precondition, but it's an atomic
			// condition. Add an additional condition so we never have an atomic
			// precondition as the main precondition.
			res.push_back(std::string("(pddl-formula"
			                          " (part-of "
			                          + parent_
			                          + ")"
			                            " (id "
			                          + name
			                          + ")"
			                            " (type conjunction)"
			                            ")"));
			// Also adapt parent and name, the parent is now the new precondition
			// above.
			new_parent = name;
			std::stringstream child_name;
			child_name << name << 1;
			name = child_name.str();
		} else {
			new_parent = parent_;
		}
		std::string params    = "";
		std::string constants = "";
		for (auto &arg : p.arguments) {
			std::vector<std::string> p_strings =
			  boost::apply_visitor(PreconditionToCLIPSFactVisitor(name, 0), arg.expression);
			if (p_strings.size() != 1) {
				throw fawkes::Exception(
				  "Parser error: Unexpected parameter length (%zu) while parsing predicate parameter in "
				  "precondition %s, expected exactly one parameter. Is '%s' really a domain predicate?",
				  p_strings.size(),
				  name.c_str(),
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
		std::string predicate_string;
		std::string predicate_string_new;
		if (p.function == "=") {
			// It's not a predicate but an equality.
			predicate_string = " (predicate EQUALITY)";
		} else {
			predicate_string = " (predicate " + p.function + ")";
		}
		// create parent atomic formula for predicate
		res.push_back(std::string("(pddl-formula"
		                          " (part-of "
		                          + new_parent
		                          + ")"
		                            " (id "
		                          + name
		                          + ")"
		                            " (type atom)"
		                            ")"));

		res.push_back(std::string("(pddl-predicate"
		                          " (part-of "
		                          + name
		                          + ")"
		                            " (id "
		                          + name + "-atom )" + predicate_string + " (param-names (create$"
		                          + params
		                          + "))"
		                            " (param-constants (create$"
		                          + constants
		                          + "))"
		                            ")"));
		return res;
	}
}
