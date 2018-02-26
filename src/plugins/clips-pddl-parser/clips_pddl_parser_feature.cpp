/***************************************************************************
 *  clips_pddl_parser_feature.cpp - CLIPS PDDL Parser Feature
 *
 *  Created: Mon 16 Oct 2017 11:14:41 CEST 11:14
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

#include "clips_pddl_parser_feature.h"
#include "precondition_visitor.h"
#include "effect_visitor.h"

#include <pddl_parser/pddl_parser.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>
#include <clipsmm.h>

#include <fstream>

using namespace std;
using namespace pddl_parser;

/** @class PDDLCLIPSFeature "clips_pddl_parser_feature.h"
 * Provide a PDDL parser to a CLIPS environment.
 * @author Till Hofmann
 */

/** Initialize the CLIPS feature.
 * @param logger The logger to use for logging in the feature
 */
PDDLCLIPSFeature::PDDLCLIPSFeature(fawkes::Logger *logger)
: CLIPSFeature("pddl-parser"), logger_(logger)
{
}

/** Initialize the context and add a parse-pddl-domain CLIPS function.
 * @param env_name The name of the environment.
 * @param clips The CLIPS environment to add the parser functionality to.
 */
void
PDDLCLIPSFeature::clips_context_init(const string &env_name,
					   fawkes::LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;
  //clips->evaluate("(path-load \"pddl.clp\")");
  clips->add_function("parse-pddl-domain",
      sigc::slot<void, string>(
        sigc::bind<0>(
          sigc::mem_fun(*this, &PDDLCLIPSFeature::parse_domain),
          env_name)));
}

/** Clean up a context.
 * @param env_name The name of the environment to clean.
 */
void
PDDLCLIPSFeature::clips_context_destroyed(const string &env_name)
{
  envs_.erase(env_name);
}

/** CLIPS function to parse a PDDL domain.
 * This parses the given domain and asserts domain facts for all parts of the
 * domain.
 * @param env_name The name of the calling environment
 * @param domain_file The path of the domain file to parse.
 */
void
PDDLCLIPSFeature::parse_domain(std::string env_name, std::string domain_file)
{
  fawkes::MutexLocker lock(envs_[env_name].objmutex_ptr());
  CLIPS::Environment &env = **(envs_[env_name]);
  Domain domain;
  try {
    ifstream df(domain_file);
    stringstream buffer;
    buffer << df.rdbuf();
    domain = PddlParser::parseDomain(buffer.str());
  } catch (PddlParserException &e) {
    logger_->log_error(("PDDLCLIPS|" + env_name).c_str(),
      "Failed to parse domain: %s", e.what_no_backtrace());
    return;
  }
  for (auto &type : domain.types) {
    string super_type = "";
    if (!type.second.empty()) {
      super_type = "(super-type " + type.second + ")";
    }
    env.assert_fact("(domain-object-type "
                    "(name " + type.first + ")"
                    + super_type +
                    ")");
  }
  for (auto &predicate : domain.predicates) {
    string param_string = "";
    string type_string = "";
    for (auto &param : predicate.second) {
      param_string += " " + param.first;
      type_string += " " + param.second;
    }
    env.assert_fact("(domain-predicate"
                    " (name " + predicate.first + ")"
                    " (param-names " + param_string + ")"
                    " (param-types " + type_string + ")"
                    ")");
  }

  for (auto &action : domain.actions) {
    string params_string = "(param-names";
    for (auto &param_pair : action.action_params) {
      string param_name = param_pair.first;
      string param_type = param_pair.second;
      params_string += " " + param_name;
      env.assert_fact("(domain-operator-parameter"
                      " (name " + param_name + ")"
                      " (operator " + action.name + ")"
                      " (type " + param_type + ")"
                      ")");
    }
    params_string += ")";
    env.assert_fact(
      "(domain-operator (name " + action.name + ")" + params_string + ")"
    );
    vector<string> precondition_facts =
      boost::apply_visitor(PreconditionToCLIPSFactVisitor(action.name, 1, true),
          action.precondition);
    for (auto &fact : precondition_facts) {
      env.assert_fact(fact);
    }
    vector<string> effect_facts =
      boost::apply_visitor(EffectToCLIPSFactVisitor(action.name, true),
          action.effect);
    for (auto &fact : effect_facts) {
      env.assert_fact(fact);
    }
  }
}
