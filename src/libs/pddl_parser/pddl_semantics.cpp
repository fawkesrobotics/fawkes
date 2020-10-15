/***************************************************************************
 *  pddl_semantics.cpp semantic checks during parsing
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

#include "pddl_semantics.h"

#include "pddl_exception.h"

#include <algorithm>

namespace pddl_parser {

pair_multi_const
ConstantSemantics::operator()(const iterator_type &   where,
                              const pair_multi_const &parsed,
                              const Domain &          domain) const
{
	// typing test:
	if (domain.requirements.end()
	    != std::find(domain.requirements.begin(), domain.requirements.end(), "typing")) {
		auto search =
		  std::find_if(domain.types.begin(), domain.types.end(), [parsed](const pair_type &p) {
			  return p.first == parsed.second || p.second == parsed.second;
		  });
		if (search == domain.types.end()) {
			throw PddlSemanticsException(std::string("Unknown type: ") + parsed.second,
			                             PddlErrorType::TYPE_ERROR,
			                             where);
		}
	}
	for (const auto &constant : parsed.first) {
		for (const auto &dom_constants : domain.constants) {
			auto already_defined =
			  std::find_if(dom_constants.first.begin(),
			               dom_constants.first.end(),
			               [parsed, constant, dom_constants](const auto &c) {
				               return c == constant && parsed.second != dom_constants.second;
			               });
			if (already_defined != dom_constants.first.end()) {
				throw PddlSemanticsException(std::string("Duplicate type: ") + constant + " type "
				                               + parsed.second + " conflicts with earlier type "
				                               + dom_constants.second,
				                             PddlErrorType::TYPE_ERROR,
				                             where);
			}
		}
	}
	return parsed;
}

Action
ActionSemantics::operator()(const iterator_type &where,
                            const Action &       parsed,
                            const Domain &       domain) const
{
	// typing test:
	// TODO: test type object
	if (domain.requirements.end()
	    != std::find(domain.requirements.begin(), domain.requirements.end(), "typing")) {
		for (const auto &action_param : parsed.action_params) {
			auto search =
			  std::find_if(domain.types.begin(), domain.types.end(), [action_param](const pair_type &p) {
				  return p.first == action_param.second || p.second == action_param.second;
			  });
			if (search == domain.types.end()) {
				throw PddlSemanticsException(std::string("Unknown type: ") + action_param.second,
				                             PddlErrorType::TYPE_ERROR,
				                             where);
			}
		}
	}
	// predicate signature test:
	check_action_predicates(where, parsed.precondition, domain, parsed);
	check_action_predicates(where, parsed.effect, domain, parsed);

	return parsed;
}

bool
ActionSemantics::check_type(const iterator_type &where,
                            const std::string &  got,
                            const std::string &  expected,
                            const Domain &       domain)
{
	if (got != expected) {
		auto generalized_it = std::find_if(domain.types.begin(),
		                                   domain.types.end(),
		                                   [got](const pair_type &p) { return p.first == got; });
		if (generalized_it == domain.types.end()) {
			return false;
		} else {
			return check_type(where, generalized_it->second, expected, domain);
		}
	} else {
		return true;
	}
}

bool
ActionSemantics::check_action_predicates(iterator_type     where,
                                         const Expression &expr,
                                         const Domain &    domain,
                                         const Action &    curr_action)
{
	bool check_types = domain.requirements.end()
	                   != std::find(domain.requirements.begin(), domain.requirements.end(), "typing");
	// this function checks predicates, if the expression is not a predicate then the action has an invalid structure
	if (boost::apply_visitor(ExpressionTypeVisitor(), expr.expression)
	    != std::type_index(typeid(Predicate))) {
		throw PddlSemanticsException(std::string("Unexpected Atom in expression: ")
		                               + boost::get<Atom>(expr.expression),
		                             PddlErrorType::EXPRESSION_ERROR,
		                             where);
	}
	Predicate pred = boost::get<Predicate>(expr.expression);
	switch (expr.type) {
	case ExpressionType::BOOL: {
		for (const auto &sub_expr : pred.arguments) {
			// recursively check sub expressions of booelean expressions, they all are predicate expressions
			check_action_predicates(where, sub_expr, domain, curr_action);
		}
		break;
	}
	case ExpressionType::PREDICATE: {
		auto defined_pred =
		  // check if the predicate name is defined in the domain ...
		  std::find_if(domain.predicates.begin(),
		               domain.predicates.end(),
		               [pred](const predicate_type &p) { return pred.function == p.first; });
		if (defined_pred == domain.predicates.end()) {
			// ... if it is not, then this predicate is invalid
			throw PddlSemanticsException(std::string("Unknown predicate: ") + pred.function,
			                             PddlErrorType::PREDICATE_ERROR,
			                             where);
		} else {
			// If the predicate is defined, the signature has to match
			if (defined_pred->second.size() != pred.arguments.size()) {
				throw PddlSemanticsException(std::string("Predicate argument length missmatch, expected ")
				                               + std::to_string(defined_pred->second.size()) + " but got "
				                               + std::to_string(pred.arguments.size()),
				                             PddlErrorType::PREDICATE_ERROR,
				                             where);
			} else {
				// and all arguments must be atomic expressions
				for (size_t i = 0; i < pred.arguments.size(); i++) {
					if (boost::apply_visitor(ExpressionTypeVisitor(), pred.arguments[i].expression)
					    != std::type_index(typeid(Atom))) {
						throw PddlSemanticsException(std::string("Unexpected nested predicate."),
						                             PddlErrorType::PREDICATE_ERROR,
						                             where);
					} else {
						Atom        curr_arg = boost::get<Atom>(pred.arguments[i].expression);
						std::string arg_type = "";
						if (curr_arg.front() != '?') {
							// constants need to be known
							auto constant_match =
							  std::find_if(domain.constants.begin(),
							               domain.constants.end(),
							               [curr_arg](const pair_multi_const &c) {
								               if (c.first.end()
								                   != std::find(c.first.begin(), c.first.end(), curr_arg)) {
									               return true;
								               } else {
									               return false;
								               }
							               });
							if (constant_match == domain.constants.end()) {
								throw PddlSemanticsException(std::string("Unknown constant ") + curr_arg,
								                             PddlErrorType::CONSTANT_ERROR,
								                             where);
							} else {
								arg_type = constant_match->second;
							}
						} else {
							auto parameter_match =
							  std::find_if(curr_action.action_params.begin(),
							               curr_action.action_params.end(),
							               [curr_arg](const pair_type &c) {
								               return c.first == curr_arg.substr(1, std::string::npos);
							               });
							if (parameter_match == curr_action.action_params.end()) {
								throw PddlSemanticsException(std::string("Unknown Parameter ") + curr_arg,
								                             PddlErrorType::PARAMETER_ERROR,
								                             where);
							} else {
								arg_type = parameter_match->second;
							}
						}
						// and if typing is required, then the types should match the signature
						if (check_types
						    && !check_type(where, arg_type, defined_pred->second[i].second, domain)) {
							throw PddlSemanticsException(std::string("Type missmatch: Argument ")
							                               + std::to_string(i) + " of " + defined_pred->first
							                               + " expects " + defined_pred->second[i].second
							                               + " but got " + arg_type,
							                             PddlErrorType::TYPE_ERROR,
							                             where);
						}
					}
				}
			}
		}
		break;
	}
	default: break;
	}
	return true;
}
} // namespace pddl_parser
