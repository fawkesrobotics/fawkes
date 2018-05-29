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

using namespace std;
using namespace pddl_parser;

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
PreconditionToCLIPSFactVisitor::PreconditionToCLIPSFactVisitor(
    const string &parent, int sub_counter, bool is_main /* = false */)
: parent_(parent), sub_counter_(sub_counter), is_main_(is_main) {}

/** Translate an Atom into a vector of strings.
 * Note that this does not return a CLIPS fact because we do not store atoms
 * (parameter names or constants) as separate facts. This needs to be further
 * processed by the caller instead.
 * @param a The atom to translate into a string.
 * @return A vector that only contains the atom as is.
 */
vector<string>
PreconditionToCLIPSFactVisitor::operator()(Atom &a) const {
  return vector<string>({a});
}

/** Translate a Predicate into a vector of strings.
 * This creates proper CLIPS precondition fact strings for the Predicate and all
 * its arguments. For compound formulae (e.g., conjunctions), this also
 * translates all sub-formulae recursively.
 * @param p The predicate to translate.
 * @return A vector of strings, each string is a properly formed CLIPS fact.
 */
vector<string>
PreconditionToCLIPSFactVisitor::operator()(Predicate &p) const {
  vector<string> res;
  stringstream namestream;
  namestream << parent_ << sub_counter_;
  string name = namestream.str();
  if (p.function == "and" || p.function == "not" || p.function == "or") {
    string type;
    if (p.function == "and") {
      type = "conjunction";
    } else if (p.function == "or") {
      type = "disjunction";
    } else if (p.function == "not") {
      type = "negation";
    }
    res.push_back(string("(domain-precondition"
                         " (name " + name + ")"
                         " (part-of " + parent_ + ")"
                         " (type " + type + ")"
                         ")"));
    uint sub_counter = 1;
    for (Expression &sub : p.arguments) {
      vector<string> args = boost::apply_visitor(
          PreconditionToCLIPSFactVisitor(name, sub_counter++), sub);
      res.insert(res.end(), args.begin(), args.end());
    }
    return res;
  } else {
    // We expect p.function to be a predicate name.
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
    for (auto &p : p.arguments) {
      vector<string> p_strings =
        boost::apply_visitor(PreconditionToCLIPSFactVisitor(name, 0), p);
      if (p_strings.size() != 1) {
        throw PddlParserException(
            "Unexpected parameter length, expected exactly one");
      }
      string p_string = p_strings[0];
      if (p_string[0] == '?') {
        // It's really a parameter.
        if (p_string.length() <= 1) {
          throw PddlParserException("Invalid parameter name " + p_string);
        }
        params += " " + p_string.substr(1);
        constants += " nil";
      } else {
        // It's a constant.
        params += " c";
        constants += " " + p_string;
      }
    }
    string predicate_string;
    if (p.function == "=") {
      // It's not a predicate but an equality.
      predicate_string = " (equality TRUE)";
    } else {
      predicate_string = " (predicate " + p.function + ")";
    }

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
}
