/***************************************************************************
 *  precondition_visitor.cpp - A static visitor to translate a precondition
 *
 *  Created: Mon 16 Oct 2017 18:34:44 CEST 18:34
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

#include "precondition_visitor.h"
#include <iostream>

using namespace std;
using namespace pddl_parser;

/** @class PreconditionToCLIPSFactVisitor "precondition_visitor.h"
 * Translate a PDDL precondition into CLIPS facts.
 * @author Till Hofmann Daniel Habering
 * Helper class to translate a precondition from pddl_parser::GoalDescription to a
 * CLIPS fact.  An GoalDescription is a boost::variant, and this class is a visitor
 * for the variant that translates the GoalDescription into a a vector of CLIPS
 * facts.
 */

/** Constructor.
 * @param parent The name of the parent (either an operator or a precondition)
 * @param sub_counter Counter passed by the parent to enumerate sub-conditions
 * @param is_main true if this is the direct child of the operator,
 * i.e., not a sub-condition
 */
PreconditionToCLIPSFactVisitor::PreconditionToCLIPSFactVisitor(const string &parent,
                                                               int           sub_counter,
                                                               bool          is_main /* = false */)
: parent_(parent), sub_counter_(sub_counter), is_main_(is_main)
{
}


/** Translate an AtomicFormula into a vector of strings.
 * This creates proper CLIPS atomic precondition fact strings for the atomic formula.
 * @param af The atomic formula to translate
 * @return A vector of strings, each string is a properly formed CLIPS fact.
 */
vector<string>
PreconditionToCLIPSFactVisitor::operator()(AtomicFormula af) const {
  vector<string> res;
  stringstream namestream;
  namestream << parent_ << sub_counter_;
  string name = namestream.str();

  string new_parent;
  if (is_main_) {
    // Special case: this is the main precondition, but it's an atomic
    // condition. Add an additional condition so we never have an atomic
    // precondition as the main precondition.
    res.push_back(string(
          "(domain-precondition"
          " (part-of " + parent_ + ")"
          " (name " + name + ")"
          " (type conjunction)"
          ")"));
    // Also adapt parent and name, the parent is now the new precondition
    // above.
    new_parent = name;
    stringstream child_name;
    child_name << name << 1;
    name = child_name.str();
  } else {
    new_parent = parent_;
  }
  string params = "";
  string constants = "";
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
  string predicate_string;
  predicate_string = " (predicate " + af.predicateName + ")";

  res.push_back(string(
        "(domain-atomic-precondition"
        " (part-of " + new_parent + ")"
        " (name " + name + ")"
        + predicate_string +
        " (param-names (create$" + params + "))"
        " (param-constants (create$" + constants + "))"
        ")"));
  return res;
}

/** Translate a FunctionalCondition into a vector of strings.
 * This creates proper CLIPS precondition fact strings for the GoalDescription and all
 * its arguments. For compound formulae (e.g., conjunctions), this also
 * translates all sub-formulae recursively.
 * @param p The predicate to translate.
 * @return A vector of strings, each string is a properly formed CLIPS fact.
 */
vector<string>
PreconditionToCLIPSFactVisitor::operator()(FunctionalCondition fc) const {
      vector<string> res;
  stringstream namestream;
  namestream << parent_ << sub_counter_;
  string name = namestream.str();
    std::string type;
    if (fc.op == OperatorFlag::EnumType::conjunction) {
      type = "conjunction";
    } else if(fc.op == OperatorFlag::EnumType::disjunction) {
      type = "disjunction";
    } else if(fc.op == OperatorFlag::EnumType::negation) {
      type = "negation";
    } else {
      throw ParserException("Unknown operator of functional condition in " + name + ": " + std::to_string(fc.op));
    }
    res.push_back(string("(domain-precondition"
                         " (name " + name + ")"
                         " (part-of " + parent_ + ")"
                         " (type " + type + ")"
                         ")"));
    uint sub_counter = 1;
    for (GoalDescription &sub : fc.condition) {
      vector<string> args = boost::apply_visitor(
          PreconditionToCLIPSFactVisitor(name, sub_counter++), sub);
      res.insert(res.end(), args.begin(), args.end());
    }
   
  return res;
}

