/***************************************************************************
 *  feature_pddl.cpp - CLIPS PDDL Feature
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

#include "feature_pddl.h"
#include "pddl_to_clips_visitor.h"

#include <pddl_parser/pddl_parser.h>
#include <core/threading/mutex_locker.h>
#include <logging/logger.h>
#include <clipsmm.h>

#include <fstream>

using namespace std;
using namespace pddl_parser;

/** @class PDDLCLIPSFeature "feature_pddl.h"
 * Provide a PDDL parser to a CLIPS environment.
 * @author Till Hofmann
 */
PDDLCLIPSFeature::PDDLCLIPSFeature(fawkes::Logger *logger)
: CLIPSFeature("pddl"), logger_(logger)
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
  } catch (PDDLParserException &e) {
    logger_->log_error(("PDDLCLIPS|" + env_name).c_str(),
      "Failed to parse domain: %s", e.what_no_backtrace());
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
    for (auto &param : predicate.second) {
      param_string += " " + param.first;
    }
    env.assert_fact("(domain-predicate "
                    "(name " + predicate.first + ") "
                    "(parameters (create$" + param_string + ")))");
  }

  for (auto &action : domain.actions) {
    env.assert_fact("(domain-operator (name " + action.name + "))");
    vector<string> facts =
      boost::apply_visitor(ExpressionToCLIPSFactVisitor(action.name, 1),
          action.precondition);
    for (auto &fact : facts) {
      env.assert_fact(fact);
    }
  }
}
