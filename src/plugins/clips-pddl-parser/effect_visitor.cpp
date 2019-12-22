/***************************************************************************
 *  effect_visitor.cpp - A static visitor to translate an effect
 *
 *  Created: Tue 31 Oct 2017 12:39:11 CET 12:39
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de> 2019 Daniel Habering
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

#include "precondition_visitor.h"

#include <iostream>

using namespace pddl_parser;

/** @class EffectToCLIPSFactVisitor "effect_visitor.h"
 * Translate a PDDL effect into CLIPS facts.
 * @author Till Hofmann, Daniel Habering
 * Helper class to translate an effect from pddl_parser::Effect to a CLIPS
 * fact.  An effect is a boost::variant, and this class is a visitor for the
 * variant that translates the Expression into a a vector of CLIPS facts.
 */

/** Constructor.
 * @param pddl_operator The name of the operator this effect belongs to.
 * @param positive True iff this is a positive (not a negative) effect.
 * @param condition Name of the condition this effect is bound to if it is an conditional effect
 * @param eff_counter Counter of all effects, used to create ids for conditional effects
 */
EffectToCLIPSFactVisitor::EffectToCLIPSFactVisitor(const std::string &pddl_operator,
                                                   bool               positive,
                                                   std::string        condition,
                                                   int                eff_counter)
: pddl_operator_(pddl_operator),
  positive_effect_(positive),
  condition_(condition),
  eff_counter_(eff_counter)
{
}

/** Translate an ConditionalEffect into a vector of strings.
 * Note that this does not return a CLIPS fact because we do not store atoms
 * (parameter names or constants) as separate facts. This visitor should not be reached,
 * since the conditional effect should be split into a goal condition and an effect,
 * which are then handled seperately.
 * @param ce The conditional effect to be parsed
 * @return An empty vector since this should not be reached by the parser.
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(ConditionalEffect &ce) const
{
	std::cout << "Should not reach this!!!!!!!!!!!!!!!!" << std::endl;
	std::vector<std::string> res;
	return res;
}

/** Translate an ActionCost into a vector of strings.
 * Action cost are not used by clips currently. Therefore, return an empty vector.
 * @param ce The ActionCost object that should be parsed
 * @return An empty vector
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(ActionCost &ce) const
{
	std::vector<std::string> res;
	return res;
}

/** Translate an AtomicFormula into a vector of strings.
 * Note that this does not return a CLIPS fact because we do not store atoms
 * (parameter names or constants) as separate facts. This needs to be further
 * processed by the caller instead.
 * @param af The AtomicFormula to be parsed.
 * @return A vector that only contains AtomicFormula as clips domain effect fact string
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(AtomicFormula &af) const
{
	std::vector<std::string> res;

	std::string params    = "";
	std::string constants = "";

	for (Term &t : af.args) {
		if (t.isVariable) {
			// It's really a parameter.
			params += " " + t.name;
			constants += " nil";
		} else {
			// It's a constant.
			params += " c";
			constants += " " + t.name;
		}
	}
	res.push_back(std::string("(domain-effect"
	                          " (part-of "
	                          + pddl_operator_
	                          + ")"
	                            " (predicate "
	                          + af.predicateName
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
	                            " (condition "
	                          + condition_
	                          + ")"
	                            ")"));
	return res;
}

/** Translate a FunctionalEffect into a vector of strings. A functional effect is
 * a construct of the form (and <effect>),(not <effect>) or (when <condition> <effect>)
 * This creates proper CLIPS effect fact strings for the Effect and all its
 * arguments. For compound formulae (e.g., conjunctions), this also translates
 * all sub-formulae recursively.
 * @param fe The FunctionalEffect to translate.
 * @return A vector of strings, each string is a properly formed CLIPS fact.
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(FunctionalEffect &fe) const
{
	std::vector<std::string> res;
	if (fe.op == pddl_parser::OperatorFlag::conjunction) {
		//Its a conjunction and the variant has to be a vector of effects
		if (fe.effect.type() != typeid(std::vector<pddl_parser::Effect>)) {
			throw ParserException(std::string("Unknown content of conjunction of " + pddl_operator_
			                                  + " expected vector<Effect>"));
		}
		std::vector<pddl_parser::Effect> effects =
		  boost::get<std::vector<pddl_parser::Effect>>(fe.effect);
		int tmp_counter = eff_counter_;
		for (auto &eff : effects) {
			std::vector<std::string> sub_effects = boost::apply_visitor(
			  EffectToCLIPSFactVisitor(pddl_operator_, positive_effect_, condition_, tmp_counter++),
			  eff.eff);
			res.insert(res.end(), sub_effects.begin(), sub_effects.end());
		}

	} else if (fe.op == pddl_parser::OperatorFlag::negation) {
		//Its a negation and the variant should contain a vector of Effect of size 1
		if (fe.effect.type() != typeid(std::vector<pddl_parser::Effect>)) {
			throw ParserException(std::string("Unknown content of negation of " + pddl_operator_
			                                  + " expected vector<Effect>"));
		}
		std::vector<pddl_parser::Effect> effects =
		  boost::get<std::vector<pddl_parser::Effect>>(fe.effect);
		// since negation is the default constructor for the operator, if the size is 0, we suspect that the effect is empty
		if (effects.size() == 0)
			return res;

		if (effects.size() != 1) {
			throw ParserException(
			  std::string("Expected exactly one sub-formula for 'not' in " + pddl_operator_));
		}
		std::vector<std::string> sub_effects = boost::apply_visitor(
		  EffectToCLIPSFactVisitor(pddl_operator_, !positive_effect_, condition_, eff_counter_),
		  effects[0].eff);
		res.insert(res.end(), sub_effects.begin(), sub_effects.end());

	} else if (fe.op == pddl_parser::OperatorFlag::condition) {
		//Its a conditional effect, the variant should be a conditional effect, which contains a goal condition and an effect
		if (fe.effect.type() != typeid(pddl_parser::ConditionalEffect)) {
			std::cout << "Type: " << fe.effect.type().name() << std::endl;
			throw ParserException(
			  std::string("Unknown content of conditional effect in " + pddl_operator_));
		}
		pddl_parser::ConditionalEffect ce = boost::get<pddl_parser::ConditionalEffect>(fe.effect);

		std::string ce_name;

		//Create an id for the conditional effect to link the condition to the effect
		//If it is a nested conditional effect, add the effect counter to the current condition id
		//Otherwise its the operater id + "-ce"
		if (condition_ == "NONE") {
			ce_name = pddl_operator_ + "-ce" + std::to_string(eff_counter_);
		} else {
			ce_name = condition_ + std::to_string(eff_counter_);
		}

		//Parse the condition
		std::vector<std::string> args =
		  boost::apply_visitor(PreconditionToCLIPSFactVisitor(ce_name, 1, false), ce.condition);
		res.insert(res.end(), args.begin(), args.end());
		//Parse the effect
		args = boost::apply_visitor(
		  EffectToCLIPSFactVisitor(pddl_operator_, positive_effect_, ce_name, eff_counter_),
		  ce.effect.eff);
		res.insert(res.end(), args.begin(), args.end());

	} else {
		throw ParserException(std::string("Unknown operator" + fe.op));
	}
	return res;
}
