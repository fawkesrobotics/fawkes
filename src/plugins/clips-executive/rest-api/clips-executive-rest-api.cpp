
/***************************************************************************
 *  clips-executive-rest-api.cpp -  CLIPS Executive REST API
 *
 *  Created: Fri Mar 16 17:17:17 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "clips-executive-rest-api.h"

#include <core/threading/mutex_locker.h>
#include <webview/rest_api_manager.h>

#include <iostream>

using namespace fawkes;

/** @class ClipsExecutiveRestApi "clips-executive-rest-api.h"
 * REST API backend for the CLIPS executive.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsExecutiveRestApi::ClipsExecutiveRestApi()
: Thread("ClipsWebviewThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
ClipsExecutiveRestApi::~ClipsExecutiveRestApi()
{
}

void
ClipsExecutiveRestApi::init()
{
	{
		std::map<std::string, LockPtr<CLIPS::Environment>> envs = clips_env_mgr->environments();
		if (envs.find("executive") == envs.end()) {
			throw Exception("No CLIPS environment named 'executive' found");
		}
		clips_ = envs["executive"];
	}

	rest_api_ = new WebviewRestApi("clips-executive", logger);
	rest_api_->add_handler<WebviewRestArray<Goal>>(
	  WebRequest::METHOD_GET, "/goals", std::bind(&ClipsExecutiveRestApi::cb_list_goals, this));
	rest_api_->add_handler<Goal>(WebRequest::METHOD_GET,
	                             "/goals/{id}",
	                             std::bind(&ClipsExecutiveRestApi::cb_get_goal,
	                                       this,
	                                       std::placeholders::_1));
	rest_api_->add_handler<WebviewRestArray<DomainOperator>>(
	  WebRequest::METHOD_GET,
	  "/domain-operators",
	  std::bind(&ClipsExecutiveRestApi::cb_list_domain_operators, this));
	rest_api_->add_handler<WebviewRestArray<DomainObject>>(
	  WebRequest::METHOD_GET,
	  "/domain-objects",
	  std::bind(&ClipsExecutiveRestApi::cb_list_domain_objects, this));
	rest_api_->add_handler<WebviewRestArray<DomainPredicate>>(
	  WebRequest::METHOD_GET,
	  "/domain-predicates",
	  std::bind(&ClipsExecutiveRestApi::cb_list_domain_predicates, this));
	rest_api_->add_handler<WebviewRestArray<DomainFact>>(
	  WebRequest::METHOD_GET,
	  "/domain-facts",
	  std::bind(&ClipsExecutiveRestApi::cb_list_domain_facts, this));
	rest_api_->add_handler<WebviewRestArray<Plan>>(
	  WebRequest::METHOD_GET, "/plans", std::bind(&ClipsExecutiveRestApi::cb_list_plans, this));
	rest_api_->add_handler<Plan>(WebRequest::METHOD_GET,
	                             "/plans/{goal-id}/{id}",
	                             std::bind(&ClipsExecutiveRestApi::cb_get_plan,
	                                       this,
	                                       std::placeholders::_1));
	rest_api_->add_handler<WebviewRestArray<PDDLGrounding>>(
	  WebRequest::METHOD_GET,
	  "/pddl-groundings",
	  std::bind(&ClipsExecutiveRestApi::cb_list_pddl_groundings, this));
	rest_api_->add_handler<WebviewRestArray<PDDLPredicate>>(
	  WebRequest::METHOD_GET,
	  "/pddl-predicates",
	  std::bind(&ClipsExecutiveRestApi::cb_list_pddl_predicates, this));
	rest_api_->add_handler<WebviewRestArray<PDDLFormula>>(
	  WebRequest::METHOD_GET,
	  "/pddl-formulas",
	  std::bind(&ClipsExecutiveRestApi::cb_list_pddl_formulas, this));
	rest_api_->add_handler<WebviewRestArray<GroundedPDDLPredicate>>(
	  WebRequest::METHOD_GET,
	  "/grounded-pddl-predicates",
	  std::bind(&ClipsExecutiveRestApi::cb_list_grounded_pddl_predicates, this));
	rest_api_->add_handler<WebviewRestArray<GroundedPDDLFormula>>(
	  WebRequest::METHOD_GET,
	  "/grounded-pddl-formulas",
	  std::bind(&ClipsExecutiveRestApi::cb_list_grounded_pddl_formulas, this));

	rest_api_->add_handler<PDDLGrounding>(WebRequest::METHOD_GET,
	                                      "/pddl-groundings/{id}",
	                                      std::bind(&ClipsExecutiveRestApi::cb_get_pddl_groundings,
	                                                this,
	                                                std::placeholders::_1));
	rest_api_->add_handler<PDDLPredicate>(WebRequest::METHOD_GET,
	                                      "/pddl-predicates/{id}",
	                                      std::bind(&ClipsExecutiveRestApi::cb_get_pddl_predicates,
	                                                this,
	                                                std::placeholders::_1));
	rest_api_->add_handler<PDDLFormula>(WebRequest::METHOD_GET,
	                                    "/pddl-formulas/{id}",
	                                    std::bind(&ClipsExecutiveRestApi::cb_get_pddl_formulas,
	                                              this,
	                                              std::placeholders::_1));
	rest_api_->add_handler<GroundedPDDLPredicate>(
	  WebRequest::METHOD_GET,
	  "/grounded-pddl-predicates/{id}",
	  std::bind(&ClipsExecutiveRestApi::cb_get_grounded_pddl_predicates,
	            this,
	            std::placeholders::_1));
	rest_api_->add_handler<GroundedPDDLFormula>(
	  WebRequest::METHOD_GET,
	  "/grounded-pddl-groundings/{id}",
	  std::bind(&ClipsExecutiveRestApi::cb_get_grounded_pddl_formulas, this, std::placeholders::_1));

	webview_rest_api_manager->register_api(rest_api_);
}

void
ClipsExecutiveRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
}

void
ClipsExecutiveRestApi::loop()
{
}

/** Get a value from a fact.
 * @param fact pointer to CLIPS fact
 * @param slot_name name of field to retrieve
 * @return template-specific return value
 */
template <typename T>
T
get_value(const CLIPS::Fact::pointer &fact, const std::string &slot_name)
{
	CLIPS::Values v = fact->slot_value(slot_name);
	if (v.empty()) {
		throw Exception("No value for slot '%s'", slot_name.c_str());
	}
	if (v[0].type() == CLIPS::TYPE_SYMBOL && v[0].as_string() == "nil") {
		return T();
	}
	return v[0];
}

/** Specialization for bool.
 * @param fact pointer to CLIPS fact
 * @param slot_name name of field to retrieve
 * @return boolean value
 */
template <>
bool
get_value(const CLIPS::Fact::pointer &fact, const std::string &slot_name)
{
	CLIPS::Values v = fact->slot_value(slot_name);
	if (v.empty()) {
		throw Exception("No value for slot '%s'", slot_name.c_str());
	}
	if (v[0].type() != CLIPS::TYPE_SYMBOL) {
		throw Exception("Value for slot '%s' is not a boolean", slot_name.c_str());
	}
	return (v[0].as_string() == "TRUE");
}

/** Get value array.
 * This is not a template because the overly verbose operator API
 * of CLIPS::Value can lead to ambiguous overloads, e.g., resolving
 * std::string to std::string or const char * operators.
 * @param fact pointer to CLIPS fact
 * @param slot_name name of field to retrieve
 * @return vector of strings from multislot
 */
static std::vector<std::string>
get_values(const CLIPS::Fact::pointer &fact, const std::string &slot_name)
{
	CLIPS::Values            v = fact->slot_value(slot_name);
	std::vector<std::string> rv(v.size());
	for (size_t i = 0; i < v.size(); ++i) {
		switch (v[i].type()) {
		case CLIPS::TYPE_FLOAT: rv[i] = std::to_string(static_cast<double>(v[i])); break;
		case CLIPS::TYPE_INTEGER: rv[i] = std::to_string(static_cast<long long int>(v[i])); break;
		case CLIPS::TYPE_SYMBOL:
		case CLIPS::TYPE_STRING:
		case CLIPS::TYPE_INSTANCE_NAME: rv[i] = static_cast<std::string &>(v[i]); break;
		default: rv[i] = "CANNOT-REPRESENT"; break;
		}
	}
	return rv;
}

Goal
ClipsExecutiveRestApi::generate_goal(CLIPS::Fact::pointer fact)
{
	Goal g;
	g.set_kind("Goal");
	g.set_apiVersion(Goal::api_version());
	g.set_id(get_value<std::string>(fact, "id"));
	g.set__class(get_value<std::string>(fact, "class"));
	g.set_type(get_value<std::string>(fact, "type"));
	g.set_sub_type(get_value<std::string>(fact, "sub-type"));
	g.set_mode(get_value<std::string>(fact, "mode"));
	g.set_parent(get_value<std::string>(fact, "parent"));
	g.set_outcome(get_value<std::string>(fact, "outcome"));
	g.set_error(get_values(fact, "error"));
	g.set_message(get_value<std::string>(fact, "message"));
	g.set_priority(get_value<long int>(fact, "priority"));
	g.set_parameters(get_values(fact, "params"));
	g.set_meta(get_values(fact, "meta"));
	g.set_required_resources(get_values(fact, "required-resources"));
	g.set_acquired_resources(get_values(fact, "acquired-resources"));

	CLIPS::Fact::pointer pfact = clips_->get_facts();
	while (pfact) {
		CLIPS::Template::pointer tmpl = pfact->get_template();
		if (tmpl->name() == "plan") {
			try {
				if (get_value<std::string>(pfact, "goal-id") == *g.id()) {
					g.addto_plans(std::move(get_value<std::string>(pfact, "id")));
				}
			} catch (Exception &e) {
			}
		}
		pfact = pfact->next();
	}

	return g;
}

WebviewRestArray<Goal>
ClipsExecutiveRestApi::cb_list_goals()
{
	MutexLocker            lock(clips_.objmutex_ptr());
	WebviewRestArray<Goal> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "goal") {
			try {
				rv.push_back(std::move(generate_goal(fact)));
			} catch (Exception &e) {
				logger->log_warn(name(), "Failed to add goal: %s", e.what_no_backtrace());
			}
		}

		fact = fact->next();
	}

	return rv;
}

Goal
ClipsExecutiveRestApi::cb_get_goal(WebviewRestParams &params)
{
	const std::string id = params.path_arg("id");

	MutexLocker          lock(clips_.objmutex_ptr());
	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "goal") {
			try {
				if (get_value<std::string>(fact, "id") == id) {
					return generate_goal(fact);
				}
			} catch (Exception &e) {
				logger->log_warn(name(), "Failed to add goal: %s", e.what_no_backtrace());
			}
		}

		fact = fact->next();
	}

	throw WebviewRestException(WebReply::HTTP_BAD_REQUEST, "Goal '%s' is unknown", id.c_str());
}

WebviewRestArray<DomainOperator>
ClipsExecutiveRestApi::cb_list_domain_operators()
{
	MutexLocker                      lock(clips_.objmutex_ptr());
	WebviewRestArray<DomainOperator> rv;

	std::map<std::string, std::list<std::pair<std::string, std::string>>> op_params;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "domain-operator-parameter") {
			std::string operator_name = get_value<std::string>(fact, "operator");
			if (op_params.find(operator_name) == op_params.end()) {
				op_params[operator_name] = {};
			}
			op_params[operator_name].push_back(
			  std::make_pair(get_value<std::string>(fact, "name"), get_value<std::string>(fact, "type")));
		}
		fact = fact->next();
	}

	fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "domain-operator") {
			try {
				DomainOperator o;
				o.set_kind("DomainOperator");
				o.set_apiVersion(DomainOperator::api_version());
				o.set_name(get_value<std::string>(fact, "name"));
				o.set_wait_sensed(get_value<bool>(fact, "wait-sensed"));
				if (op_params.find(*o.name()) != op_params.end()) {
					for (const auto &p : op_params[*o.name()]) {
						DomainOperatorParameter param;
						param.set_name(p.first);
						param.set_type(p.second);
						o.addto_parameters(std::move(param));
					}
				}

				rv.push_back(std::move(o));
			} catch (Exception &e) {
				logger->log_warn(name(), "Failed to add goal: %s", e.what_no_backtrace());
			}
		}

		fact = fact->next();
	}

	return rv;
}

WebviewRestArray<DomainObject>
ClipsExecutiveRestApi::cb_list_domain_objects()
{
	MutexLocker                    lock(clips_.objmutex_ptr());
	WebviewRestArray<DomainObject> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "domain-object") {
			DomainObject o;
			o.set_kind("DomainObject");
			o.set_apiVersion(DomainObject::api_version());
			o.set_name(get_value<std::string>(fact, "name"));
			o.set_type(get_value<std::string>(fact, "type"));
			rv.push_back(std::move(o));
		}
		fact = fact->next();
	}

	return rv;
}

WebviewRestArray<DomainPredicate>
ClipsExecutiveRestApi::cb_list_domain_predicates()
{
	MutexLocker                       lock(clips_.objmutex_ptr());
	WebviewRestArray<DomainPredicate> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "domain-predicate") {
			DomainPredicate p;
			p.set_kind("DomainPredicate");
			p.set_apiVersion(DomainPredicate::api_version());
			p.set_name(get_value<std::string>(fact, "name"));
			p.set_sensed(get_value<bool>(fact, "sensed"));
			p.set_param_names(get_values(fact, "param-names"));
			p.set_param_types(get_values(fact, "param-types"));
			rv.push_back(std::move(p));
		}
		fact = fact->next();
	}

	return rv;
}

WebviewRestArray<DomainFact>
ClipsExecutiveRestApi::cb_list_domain_facts()
{
	MutexLocker                  lock(clips_.objmutex_ptr());
	WebviewRestArray<DomainFact> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "domain-fact") {
			DomainFact f;
			f.set_kind("DomainFact");
			f.set_apiVersion(DomainFact::api_version());
			f.set_name(get_value<std::string>(fact, "name"));
			f.set_param_values(get_values(fact, "param-values"));
			rv.push_back(std::move(f));
		}
		fact = fact->next();
	}

	return rv;
}

std::shared_ptr<PDDLGrounding>
ClipsExecutiveRestApi::gen_pddl_grounding(const CLIPS::Fact::pointer fact)
{
	auto grounding = std::make_shared<PDDLGrounding>();
	grounding->set_kind("PDDLGrounding");
	grounding->set_apiVersion(PDDLGrounding::api_version());
	grounding->set_id(get_value<std::string>(fact, "id"));

	for (const auto &pn : get_values(fact, "param-names")) {
		grounding->addto_param_names(std::move(pn));
	}

	for (const auto &pv : get_values(fact, "param-values")) {
		grounding->addto_param_values(std::move(pv));
	}

	return grounding;
}

WebviewRestArray<PDDLGrounding>
ClipsExecutiveRestApi::cb_list_pddl_groundings()
{
	MutexLocker                     lock(clips_.objmutex_ptr());
	WebviewRestArray<PDDLGrounding> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "pddl-grounding") {
			rv.push_back(*gen_pddl_grounding(fact));
		}
		fact = fact->next();
	}

	return rv;
}

PDDLGrounding
ClipsExecutiveRestApi::cb_get_pddl_groundings(WebviewRestParams &params)
{
	std::string id = params.path_arg("id");

	MutexLocker   lock(clips_.objmutex_ptr());
	PDDLGrounding ret;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "pddl-grounding" && get_value<std::string>(fact, "id") == id) {
			ret = *gen_pddl_grounding(fact);
		}
		fact = fact->next();
	}

	throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
	                           "No grounding with ID '%s' found",
	                           id.c_str());

	return ret;
}

std::shared_ptr<PDDLFormula>
ClipsExecutiveRestApi::gen_pddl_formula(const CLIPS::Fact::pointer fact)
{
	auto formula = std::make_shared<PDDLFormula>();
	formula->set_kind("PDDLFormula");
	formula->set_apiVersion(PDDLFormula::api_version());
	formula->set_id(get_value<std::string>(fact, "id"));
	formula->set_type(get_value<std::string>(fact, "type"));
	formula->set_part_of(get_value<std::string>(fact, "part-of"));

	return formula;
}

WebviewRestArray<PDDLFormula>
ClipsExecutiveRestApi::cb_list_pddl_formulas()
{
	MutexLocker                   lock(clips_.objmutex_ptr());
	WebviewRestArray<PDDLFormula> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "pddl-formula") {
			rv.push_back(*gen_pddl_formula(fact));
		}
		fact = fact->next();
	}

	return rv;
}

PDDLFormula
ClipsExecutiveRestApi::cb_get_pddl_formulas(WebviewRestParams &params)
{
	std::string id = params.path_arg("id");

	MutexLocker lock(clips_.objmutex_ptr());
	PDDLFormula ret;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "pddl-formula" && get_value<std::string>(fact, "id") == id) {
			ret = *gen_pddl_formula(fact);
		}
		fact = fact->next();
	}

	throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
	                           "No formula with ID '%s' found",
	                           id.c_str());

	return ret;
}

std::shared_ptr<PDDLPredicate>
ClipsExecutiveRestApi::gen_pddl_predicate(const CLIPS::Fact::pointer fact)
{
	auto predicate = std::make_shared<PDDLPredicate>();
	predicate->set_kind("PDDLPredicate");
	predicate->set_apiVersion(PDDLPredicate::api_version());
	predicate->set_id(get_value<std::string>(fact, "id"));
	predicate->set_part_of(get_value<std::string>(fact, "part-of"));
	predicate->set_predicate(get_value<std::string>(fact, "predicate"));

	for (const auto &pn : get_values(fact, "param-names")) {
		predicate->addto_param_names(std::move(pn));
	}

	for (const auto &pc : get_values(fact, "param-constants")) {
		predicate->addto_param_constants(std::move(pc));
	}

	return predicate;
}

WebviewRestArray<PDDLPredicate>
ClipsExecutiveRestApi::cb_list_pddl_predicates()
{
	MutexLocker                     lock(clips_.objmutex_ptr());
	WebviewRestArray<PDDLPredicate> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "pddl-predicate") {
			rv.push_back(*gen_pddl_predicate(fact));
		}
		fact = fact->next();
	}

	return rv;
}

PDDLPredicate
ClipsExecutiveRestApi::cb_get_pddl_predicates(WebviewRestParams &params)
{
	std::string id = params.path_arg("id");

	MutexLocker   lock(clips_.objmutex_ptr());
	PDDLPredicate ret;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "pddl-predicate" && get_value<std::string>(fact, "id") == id) {
			ret = *gen_pddl_predicate(fact);
		}
		fact = fact->next();
	}

	throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
	                           "No predicate with ID '%s' found",
	                           id.c_str());

	return ret;
}

std::shared_ptr<GroundedPDDLFormula>
ClipsExecutiveRestApi::gen_grounded_pddl_formula(const CLIPS::Fact::pointer fact)
{
	auto formula = std::make_shared<GroundedPDDLFormula>();
	formula->set_kind("GroundedPDDLFormula");
	formula->set_apiVersion(GroundedPDDLFormula::api_version());
	formula->set_id(get_value<std::string>(fact, "id"));
	formula->set_formula_id(get_value<std::string>(fact, "formula-id"));
	formula->set_grounding(get_value<std::string>(fact, "grounding"));
	formula->set_is_satisfied(get_value<bool>(fact, "is-satisfied"));

	return formula;
}

WebviewRestArray<GroundedPDDLFormula>
ClipsExecutiveRestApi::cb_list_grounded_pddl_formulas()
{
	MutexLocker                           lock(clips_.objmutex_ptr());
	WebviewRestArray<GroundedPDDLFormula> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "grounded-pddl-formula") {
			rv.push_back(*gen_grounded_pddl_formula(fact));
		}
		fact = fact->next();
	}

	return rv;
}

GroundedPDDLFormula
ClipsExecutiveRestApi::cb_get_grounded_pddl_formulas(WebviewRestParams &params)
{
	std::string id = params.path_arg("id");

	MutexLocker         lock(clips_.objmutex_ptr());
	GroundedPDDLFormula ret;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "grounded-pddl-formula" && get_value<std::string>(fact, "id") == id) {
			ret = *gen_grounded_pddl_formula(fact);
		}
		fact = fact->next();
	}

	throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
	                           "No grounded formula with ID '%s' found",
	                           id.c_str());

	return ret;
}

std::shared_ptr<GroundedPDDLPredicate>
ClipsExecutiveRestApi::gen_grounded_pddl_predicate(const CLIPS::Fact::pointer fact)
{
	auto predicate = std::make_shared<GroundedPDDLPredicate>();
	predicate->set_kind("GroundedPDDLPredicate");
	predicate->set_apiVersion(GroundedPDDLPredicate::api_version());
	predicate->set_id(get_value<std::string>(fact, "id"));
	predicate->set_predicate_id(get_value<std::string>(fact, "predicate-id"));
	predicate->set_grounding(get_value<std::string>(fact, "grounding"));
	predicate->set_is_satisfied(get_value<bool>(fact, "is-satisfied"));

	return predicate;
}

WebviewRestArray<GroundedPDDLPredicate>
ClipsExecutiveRestApi::cb_list_grounded_pddl_predicates()
{
	MutexLocker                             lock(clips_.objmutex_ptr());
	WebviewRestArray<GroundedPDDLPredicate> rv;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "grounded-pddl-predicate") {
			rv.push_back(*gen_grounded_pddl_predicate(fact));
		}
		fact = fact->next();
	}

	return rv;
}

GroundedPDDLPredicate
ClipsExecutiveRestApi::cb_get_grounded_pddl_predicates(WebviewRestParams &params)
{
	std::string id = params.path_arg("id");

	MutexLocker           lock(clips_.objmutex_ptr());
	GroundedPDDLPredicate ret;

	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "grounded-pddl-predicate" && get_value<std::string>(fact, "id") == id) {
			ret = *gen_grounded_pddl_predicate(fact);
		}
		fact = fact->next();
	}

	throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
	                           "No grounded predicate with ID '%s' found",
	                           id.c_str());

	return ret;
}

void
ClipsExecutiveRestApi::gen_plan_precompute(PlanMap &                 plans,
                                           PlanActionMap &           plan_actions,
                                           PreCompoundMap &          prec,
                                           PreAtomMap &              prea,
                                           PDDLGroundingMap &        pgm,
                                           PDDLFormulaMap &          pfm,
                                           PDDLPredicateMap &        ppm,
                                           GroundedPDDLFormulaMap &  gpfm,
                                           GroundedPDDLPredicateMap &gppm)
{
	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "plan") {
			plans[std::make_pair(get_value<std::string>(fact, "goal-id"),
			                     get_value<std::string>(fact, "id"))] = fact;
		} else if (tmpl->name() == "plan-action") {
			plan_actions[std::make_pair(get_value<std::string>(fact, "goal-id"),
			                            get_value<std::string>(fact, "plan-id"))]
			  .push_back(fact);
		} else if (tmpl->name() == "pddl-grounding") {
			pgm[get_value<std::string>(fact, "id")] = fact;
		} else if (tmpl->name() == "pddl-formula") {
			pfm[get_value<std::string>(fact, "id")] = (fact);
		} else if (tmpl->name() == "pddl-predicate") {
			ppm[get_value<std::string>(fact, "id")] = (fact);
		} else if (tmpl->name() == "grounded-pddl-formula") {
			gpfm[get_value<std::string>(fact, "grounding")].push_back(fact);
		} else if (tmpl->name() == "grounded-pddl-predicate") {
			gppm[get_value<std::string>(fact, "grounding")].push_back(fact);
		}
		fact = fact->next();
	}
}

GroundedFormula
ClipsExecutiveRestApi::gen_plan_compute_precons(PDDLFormulaTreeNode node,
                                                PDDLFormulaTreeMap  tree,
                                                PDDLGroundingMap    groundings)
{
	CLIPS::Fact::pointer     node_fp   = std::get<0>(node);
	CLIPS::Fact::pointer     node_g_fp = std::get<1>(node);
	CLIPS::Template::pointer tmpl      = node_fp->get_template();
	GroundedFormula          formula;

	formula.set_apiVersion(GroundedFormula::api_version());
	if (tmpl->name() == "pddl-predicate") {
		formula.set_kind("GroundedPredicate");
		formula.set_name(get_value<std::string>(node_fp, "predicate"));
		formula.set_type("atom");
		std::vector<std::string> param_names =
		  get_values(groundings[get_value<std::string>(node_g_fp, "grounding")], "param-names");
		std::vector<std::string> param_values =
		  get_values(groundings[get_value<std::string>(node_g_fp, "grounding")], "param-values");
		std::vector<std::string> predicate_param_values;
		for (const auto &param : get_values(node_fp, "param-names")) {
			auto it = std::find(param_names.begin(), param_names.end(), param);
			if (it != param_names.end()) {
				auto index = std::distance(param_names.begin(), it);
				predicate_param_values.push_back(param_values[index]);
			}
		}
		formula.set_param_names(get_values(node_g_fp, "param-names"));
		formula.set_param_values(predicate_param_values);
		formula.set_param_constants(get_values(node_g_fp, "param-constants"));
	} else if (tmpl->name() == "pddl-formula") {
		formula.set_kind("GroundedFormula");
		formula.set_name(get_value<std::string>(node_fp, "id"));
		formula.set_type(get_value<std::string>(node_fp, "type"));
	}
	get_value<std::string>(node_g_fp, "is-satisfied") == "TRUE" ? formula.set_is_satisfied(true)
	                                                            : formula.set_is_satisfied(false);

	for (const auto &child_node : tree[get_value<std::string>(node_fp, "id")]) {
		auto child = get_value<std::string>(std::get<0>(child_node), "id");
		formula.addto_child(gen_plan_compute_precons(child_node, tree, groundings));
	}

	return formula;
}

Plan
ClipsExecutiveRestApi::gen_plan(const PlanKey &            plan_key,
                                const CLIPS::Fact::pointer fact,
                                PlanActionMap &            plan_actions,
                                PreCompoundMap &           prec,
                                PreAtomMap &               prea,
                                PDDLGroundingMap &         pgm,
                                PDDLFormulaMap &           pfm,
                                PDDLPredicateMap &         ppm,
                                GroundedPDDLFormulaMap &   gpfm,
                                GroundedPDDLPredicateMap & gppm)
{
	const std::string &goal_id = get_value<std::string>(fact, "goal-id");
	const std::string &plan_id = get_value<std::string>(fact, "id");

	Plan p;
	p.set_kind("Plan");
	p.set_apiVersion(Plan::api_version());
	p.set_goal_id(goal_id);
	p.set_id(plan_id);
	p.set_cost(get_value<double>(fact, "cost"));
	if (plan_actions.find(plan_key) != plan_actions.end()) {
		std::vector<std::shared_ptr<PlanAction>> actions;

		for (auto &pai : plan_actions[plan_key]) {
			auto pa = std::make_shared<PlanAction>();

			int64_t     action_id     = get_value<int64_t>(pai, "id");
			std::string operator_name = get_value<std::string>(pai, "action-name");

			// general info
			pa->set_kind("PlanAction");
			pa->set_apiVersion(PlanAction::api_version());
			pa->set_id(action_id);
			pa->set_operator_name(operator_name);
			for (const auto &pv : get_values(pai, "param-values")) {
				pa->addto_param_values(std::move(pv));
			}
			pa->set_state(get_value<std::string>(pai, "state"));
			pa->set_executable(get_value<bool>(pai, "executable"));
			pa->set_duration(get_value<double>(pai, "duration"));
			pa->set_dispatch_time(get_value<double>(pai, "dispatch-time"));

			// preconditions
			if (get_value<std::string>(pai, "precondition") != "") {
				pa->set_precondition(gen_pddl_grounding(pgm[get_value<std::string>(pai, "precondition")]));

				std::string        grnd_name = *(pa->precondition()->id());
				std::string        op_name   = *(pa->operator_name());
				PDDLFormulaTreeMap grounded_parent_map;

				for (const auto &p : gpfm[grnd_name]) {
					PDDLFormulaTreeNode node =
					  std::make_tuple(pfm[get_value<std::string>(p, "formula-id")], p);
					grounded_parent_map[get_value<std::string>(pfm[get_value<std::string>(p, "formula-id")],
					                                           "part-of")]
					  .push_back(node);
				}
				for (const auto &p : gppm[grnd_name]) {
					PDDLFormulaTreeNode node =
					  std::make_tuple(ppm[get_value<std::string>(p, "predicate-id")], p);
					grounded_parent_map[get_value<std::string>(ppm[get_value<std::string>(p, "predicate-id")],
					                                           "part-of")]
					  .push_back(node);
				}

				//get root node
				for (const auto &rnode : grounded_parent_map[op_name]) {
					CLIPS::Fact::pointer root_fp = std::get<0>(rnode);
					std::string          root    = get_value<std::string>(root_fp, "id");

					//recursively compute the tree and set the preconditions
					pa->set_preconditions(std::make_shared<GroundedFormula>(
					  gen_plan_compute_precons(rnode, grounded_parent_map, pgm)));
				}
			}

			actions.push_back(std::move(pa));
		}

		std::sort(actions.begin(),
		          actions.end(),
		          [](std::shared_ptr<PlanAction> &a, std::shared_ptr<PlanAction> &b) {
			          return *a->id() < *b->id();
		          });
		p.set_actions(actions);
	}

	return p;
}

WebviewRestArray<Plan>
ClipsExecutiveRestApi::cb_list_plans()
{
	MutexLocker            lock(clips_.objmutex_ptr());
	WebviewRestArray<Plan> rv;

	std::map<PlanKey, CLIPS::Fact::pointer> plans;
	std::map<PlanKey, ClipsFactList>        plan_actions;
	PreCompoundMap                          prec;
	PreAtomMap                              prea;
	PDDLGroundingMap                        pgm;
	PDDLFormulaMap                          pfm;
	PDDLPredicateMap                        ppm;
	GroundedPDDLFormulaMap                  gpfm;
	GroundedPDDLPredicateMap                gppm;

	gen_plan_precompute(plans, plan_actions, prec, prea, pgm, pfm, ppm, gpfm, gppm);

	for (auto &pi : plans) {
		rv.push_back(std::move(
		  gen_plan(pi.first, pi.second, plan_actions, prec, prea, pgm, pfm, ppm, gpfm, gppm)));
	}

	return rv;
}

Plan
ClipsExecutiveRestApi::cb_get_plan(WebviewRestParams &params)
{
	std::string goal_id = params.path_arg("goal-id");
	std::string id      = params.path_arg("id");

	MutexLocker            lock(clips_.objmutex_ptr());
	WebviewRestArray<Plan> rv;

	std::map<PlanKey, CLIPS::Fact::pointer> plans;
	std::map<PlanKey, ClipsFactList>        plan_actions;
	PreCompoundMap                          prec;
	PreAtomMap                              prea;
	PDDLGroundingMap                        pgm;
	PDDLFormulaMap                          pfm;
	PDDLPredicateMap                        ppm;
	GroundedPDDLFormulaMap                  gpfm;
	GroundedPDDLPredicateMap                gppm;

	gen_plan_precompute(plans, plan_actions, prec, prea, pgm, pfm, ppm, gpfm, gppm);

	const PlanKey plan_key{goal_id, id};
	if (plans.find(plan_key) == plans.end()) {
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
		                           "No plan for goal '%s' with ID '%s' found",
		                           goal_id.c_str(),
		                           id.c_str());
	}

	return gen_plan(plan_key, plans[plan_key], plan_actions, prec, prea, pgm, pfm, ppm, gpfm, gppm);
}
