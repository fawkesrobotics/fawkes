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
#include "precondition_visitor.h"
#include <iostream>

using namespace pddl_parser;

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
EffectToCLIPSFactVisitor::EffectToCLIPSFactVisitor(
    const std::string &pddl_operator, bool positive, std::string condition)
: pddl_operator_(pddl_operator), positive_effect_(positive), condition_(condition) {}

/** Translate an ConditionalEffect into a vector of strings.
 * Note that this does not return a CLIPS fact because we do not store atoms
 * (parameter names or constants) as separate facts. This needs to be further
 * processed by the caller instead.
 * @param a The atom to translate into a string.
 * @return A vector that only contains the atom as is.
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(ConditionalEffect &ce) const {
  std::vector<std::string> res;
  return res;
}

/** Translate an ActionCost into a vector of strings.
 * Note that this does not return a CLIPS fact because we do not store atoms
 * (parameter names or constants) as separate facts. This needs to be further
 * processed by the caller instead.
 * @param a The atom to translate into a string.
 * @return A vector that only contains the atom as is.
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(ActionCost &ce) const {
  std::vector<std::string> res;
  std::string cost_name = ce.name;
  int cost = ce.cost;
  res.push_back(std::string(
          "(domain-action-cost"
          " (part-of " + pddl_operator_ + ")"
          " (cost-name " + cost_name + ")"
          " (cost " + std::to_string(cost) + ")"
          ")"));
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
EffectToCLIPSFactVisitor::operator()(AtomicFormula &af) const {
    std::vector<std::string> res;
  // We expect p.function to be a predicate name.
    std::string params = "";
    std::string constants = "";
    for (Term &t: af.args) {
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
    res.push_back(std::string(
          "(domain-effect"
          " (part-of " + pddl_operator_ + ")"
          " (predicate " + af.predicateName + ")"
          " (param-names " + params + ")"
          " (param-constants " + constants + ")"
          " (type " + (positive_effect_ ? "POSITIVE" : "NEGATIVE") + ")"
          " (condition " + condition_ + ")"
          ")"));
    return res;
}

/** Translate a Predicate into a vector of strings.
 * This creates proper CLIPS effect fact strings for the Predicate and all its
 * arguments. For compound formulae (e.g., conjunctions), this also translates
 * all sub-formulae recursively.
 * @param p The predicate to translate.
 * @return A vector of strings, each string is a properly formed CLIPS fact.
 */
std::vector<std::string>
EffectToCLIPSFactVisitor::operator()(FunctionalEffect &fe) const {
  std::vector<std::string> res;
  if (fe.op == pddl_parser::OperatorFlag::conjunction) {
    if (fe.effect.type() != typeid(std::vector<pddl_parser::Effect>)){
      throw ParserException("Unknown content of conjunction");
    }
    std::vector<pddl_parser::Effect> effects = boost::get<std::vector<pddl_parser::Effect>>(fe.effect);
    for (auto &eff : effects) {
      std::vector<std::string> sub_effects = boost::apply_visitor(
          EffectToCLIPSFactVisitor(pddl_operator_, positive_effect_,condition_), eff.eff);
      res.insert(res.end(), sub_effects.begin(), sub_effects.end());
    }
  } else if (fe.op == pddl_parser::OperatorFlag::negation) {
    if (fe.effect.type() != typeid(std::vector<pddl_parser::Effect>)){
      throw ParserException("Unknown content of conjunction");
    }
    std::vector<pddl_parser::Effect> effects = boost::get<std::vector<pddl_parser::Effect>>(fe.effect);
    if (effects.size() == 0) {
      return res;
    }
    if (effects.size() != 1) {
      throw ParserException("Expected exactly one sub-formula for 'not'");
    }
    std::vector<std::string> sub_effects = boost::apply_visitor(
        EffectToCLIPSFactVisitor(pddl_operator_, !positive_effect_,condition_),
        effects[0].eff);
    res.insert(res.end(), sub_effects.begin(), sub_effects.end());
  } else if (fe.op == pddl_parser::OperatorFlag::condition) {
    if (fe.effect.type() != typeid(pddl_parser::ConditionalEffect)){
      throw ParserException("Unknown content of conditional effect");
    }
    pddl_parser::ConditionalEffect ce = boost::get<pddl_parser::ConditionalEffect>(fe.effect);
    std::string ce_name;
    if (condition_ == "NONE") {
      ce_name = pddl_operator_ + "-ce";
    } else {
      ce_name = condition_ + "-ce";
    }
    std::vector<std::string> args = boost::apply_visitor(
          PreconditionToCLIPSFactVisitor(ce_name, 1, false), ce.condition);
    res.insert(res.end(), args.begin(), args.end());
    args = boost::apply_visitor(
          EffectToCLIPSFactVisitor(pddl_operator_, positive_effect_,ce_name), ce.effect.eff);
    res.insert(res.end(), args.begin(), args.end());

  } else {
    throw ParserException(std::string("Unknown operator" + fe.op));
  }
  return res;
}
