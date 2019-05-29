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

#include "clips_pddl_parser_thread.h"
#include <pddl_parser/pddl_parser.h>
#include <core/threading/mutex_locker.h>
#include "precondition_visitor.h"
#include "effect_visitor.h"

#include <core/threading/mutex_locker.h>

#include <iostream>
#include <fstream>

//using namespace std;
//using namespace pddl_parser;

using namespace fawkes;

/** @class PDDLCLIPSFeature "clips_pddl_parser_feature.h"
 * Provide a PDDL parser to a CLIPS environment.
 * @author Till Hofmann
 */

/** Initialize the CLIPS feature.
 * @param logger The logger to use for logging in the feature
 */
PDDLCLIPSFeature::PDDLCLIPSFeature()
: Thread("ClipsPddlParser", Thread::OPMODE_WAITFORWAKEUP),
  CLIPSFeature("pddl-parser"), CLIPSFeatureAspect(this)
{
}


void
PDDLCLIPSFeature::init()
{
}

void
PDDLCLIPSFeature::loop()
{
}

void
PDDLCLIPSFeature::finalize()
{
  envs_.clear();
}

void
PDDLCLIPSFeature::clips_context_init(const std::string &env_name,
					   fawkes::LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;
  //clips->evaluate("(path-load \"pddl.clp\")");
  clips->add_function("parse-pddl-domain",
      sigc::slot<void, std::string>(
        sigc::bind<0>(
          sigc::mem_fun(*this, &PDDLCLIPSFeature::parse_domain),
          env_name)));
}

/** Clean up a context.
 * @param env_name The name of the environment to clean.
 */
void
PDDLCLIPSFeature::clips_context_destroyed(const std::string &env_name)
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
  pddl_parser::PddlDomain domain;
  pddl_parser::Parser parser;
  try {
    logger->log_info(name(),"Starting pddl parsing for %s on %s",env_name.c_str(),domain_file.c_str());
    std::ifstream df(domain_file);
    std::stringstream buffer;
    buffer << df.rdbuf();
    domain = parser.parseDomain(buffer.str());
  } catch (pddl_parser::ParserException &e) {
    logger->log_error(("PDDLCLIPS|" + env_name).c_str(),
      "Failed to parse domain: %s", e.what());
    return;
  }
  for (auto &type : domain.types) {
    std::string super_type = "";
    if (!type.type.empty()) {
      super_type = "(super-type " + type.type + ")";
    }
    env.assert_fact("(domain-object-type "
                    "(name " + type.name + ")"
                    + super_type +
                    ")");
  }
  for (auto &predicate : domain.predicates) {
    std::string param_string = "";
    std::string type_string = "";
    for (auto &param : predicate.second) {
      param_string += " " + param.name;
      type_string += " " + param.type;
    }
    env.assert_fact("(domain-predicate"
                    " (name " + predicate.first + ")"
                    " (param-names " + param_string + ")"
                    " (param-types " + type_string + ")"
                    ")");
  }

  for (auto &action : domain.actions) {
    std::string params_string = "(param-names";
    for (auto &param_pair : action.parameters) {
      std::string param_name = param_pair.name;
      std::string param_type = param_pair.type;
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
    
    logger->log_info(name(), "Parsing %s",action.name.c_str());
    std::vector<std::string> precondition_facts = 
      boost::apply_visitor(PreconditionToCLIPSFactVisitor(action.name, 1, true),
          action.precondition);
    for (auto &fact : precondition_facts) {
      logger->log_info(name(),fact.c_str());
      env.assert_fact(fact); 
    }  
    std::vector<std::string> effect_facts =
      boost::apply_visitor(EffectToCLIPSFactVisitor(action.name, true, "NONE"),
          action.effect.eff);
    for (auto &fact : effect_facts) {
      env.assert_fact(fact);
    } 
  }
}
