
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

#include <webview/rest_api_manager.h>
#include <core/threading/mutex_locker.h>

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
	rest_api_->add_handler<WebviewRestArray<Goal>>
		(WebRequest::METHOD_GET, "/goals",
		 std::bind(&ClipsExecutiveRestApi::cb_list_goals, this));
	rest_api_->add_handler<Goal>
		(WebRequest::METHOD_GET, "/goals/{id}",
		 std::bind(&ClipsExecutiveRestApi::cb_get_goal, this, std::placeholders::_1));
	rest_api_->add_handler<WebviewRestArray<DomainOperator>>
		(WebRequest::METHOD_GET, "/domain-operators",
		 std::bind(&ClipsExecutiveRestApi::cb_list_domain_operators, this));
	rest_api_->add_handler<WebviewRestArray<DomainObject>>
		(WebRequest::METHOD_GET, "/domain-objects",
		 std::bind(&ClipsExecutiveRestApi::cb_list_domain_objects, this));
	rest_api_->add_handler<WebviewRestArray<DomainPredicate>>
		(WebRequest::METHOD_GET, "/domain-predicates",
		 std::bind(&ClipsExecutiveRestApi::cb_list_domain_predicates, this));
	rest_api_->add_handler<WebviewRestArray<DomainFact>>
		(WebRequest::METHOD_GET, "/domain-facts",
		 std::bind(&ClipsExecutiveRestApi::cb_list_domain_facts, this));
	rest_api_->add_handler<WebviewRestArray<Plan>>
		(WebRequest::METHOD_GET, "/plans",
		 std::bind(&ClipsExecutiveRestApi::cb_list_plans, this));
	rest_api_->add_handler<Plan>
		(WebRequest::METHOD_GET, "/plans/{goal-id}/{id}",
		 std::bind(&ClipsExecutiveRestApi::cb_get_plan, this, std::placeholders::_1));
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
T get_value(const CLIPS::Fact::pointer &fact, const std::string &slot_name)
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
bool get_value(const CLIPS::Fact::pointer &fact, const std::string &slot_name)
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
	CLIPS::Values v = fact->slot_value(slot_name);
	std::vector<std::string> rv(v.size());
	for (size_t i = 0; i < v.size(); ++i) {
		rv[i] = static_cast<std::string&>(v[i]);
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
	g.set_mode(get_value<std::string>(fact, "mode"));
	g.set_outcome(get_value<std::string>(fact, "outcome"));
	g.set_parent(get_value<std::string>(fact, "parent"));
	g.set_message(get_value<std::string>(fact, "message"));
	g.set_priority(get_value<long int>(fact, "priority"));
	g.set_parameters(get_values(fact, "params"));

	CLIPS::Fact::pointer pfact = clips_->get_facts();
	while (pfact) {
		CLIPS::Template::pointer tmpl = pfact->get_template();
		if (tmpl->name() == "plan") {
			try {
				if (get_value<std::string>(pfact, "goal-id") == *g.id()) {
					g.addto_plans(std::move(get_value<std::string>(pfact, "id")));
				}
			} catch (Exception &e) {}
		}
		pfact = pfact->next();
	}

	return g;
}

WebviewRestArray<Goal>
ClipsExecutiveRestApi::cb_list_goals()
{
	MutexLocker lock(clips_.objmutex_ptr());
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
ClipsExecutiveRestApi::cb_get_goal(WebviewRestParams& params)
{
	const std::string id = params.path_arg("id");
	
	MutexLocker lock(clips_.objmutex_ptr());
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
	
	throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
	                           "Goal '%s' is unknown", id.c_str());
}

WebviewRestArray<DomainOperator>
ClipsExecutiveRestApi::cb_list_domain_operators()
{
	MutexLocker lock(clips_.objmutex_ptr());
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
			op_params[operator_name].push_back(std::make_pair(get_value<std::string>(fact, "name"),
			                                                  get_value<std::string>(fact, "type")));
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
	MutexLocker lock(clips_.objmutex_ptr());
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
	MutexLocker lock(clips_.objmutex_ptr());
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
	MutexLocker lock(clips_.objmutex_ptr());
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


std::shared_ptr<DomainPreconditionAtom>
ClipsExecutiveRestApi::gen_domain_precondition_atom(const CLIPS::Fact::pointer fact)
{
	auto pre_atom = std::make_shared<DomainPreconditionAtom>();
	pre_atom->set_kind("DomainPreconditionAtom");
	pre_atom->set_apiVersion(DomainPreconditionAtom::api_version());
	pre_atom->set_name(get_value<std::string>(fact, "name"));
	pre_atom->set_type("atom");
	pre_atom->set_grounded(get_value<bool>(fact, "grounded"));
	pre_atom->set_is_satisfied(get_value<bool>(fact, "is-satisfied"));
	pre_atom->set_predicate(get_value<std::string>(fact, "predicate"));
	for (const auto& s : get_values(fact, "param-names")) {
		pre_atom->addto_param_names(std::move(s));
	}
	for (const auto& s : get_values(fact, "param-values")) {
		pre_atom->addto_param_values(std::move(s));
	}
	for (const auto& s : get_values(fact, "param-constants")) {
		pre_atom->addto_param_constants(std::move(s));
	}
	return pre_atom;
}

std::shared_ptr<DomainPreconditionCompound>
ClipsExecutiveRestApi::gen_domain_precondition_compound(const CLIPS::Fact::pointer fact,
                                                        const PlanActionKey &plan_action_key,
                                                        PreCompoundMap &prec, PreAtomMap &prea)
{
	std::string prec_name = get_value<std::string>(fact, "name");

	auto pre_comp = std::make_shared<DomainPreconditionCompound>();
	pre_comp->set_kind("DomainPreconditionCompound");
	pre_comp->set_apiVersion(DomainPreconditionCompound::api_version());
	pre_comp->set_name(prec_name);
	pre_comp->set_type(get_value<std::string>(fact, "type"));
	pre_comp->set_grounded(get_value<bool>(fact, "grounded"));
	pre_comp->set_is_satisfied(get_value<bool>(fact, "is-satisfied"));

	// elements of pre_compondition compound
	for (const auto& prea_fact : prea[plan_action_key]) {
		std::string part_of = get_value<std::string>(prea_fact, "part-of");
		if (part_of == prec_name) {
			pre_comp->addto_elements(gen_domain_precondition_atom(prea_fact));
		}
	}
	for (const auto& prec_fact : prec[plan_action_key]) {
		std::string part_of = get_value<std::string>(prec_fact, "part-of");
		if (part_of == prec_name) {
			pre_comp->addto_elements(gen_domain_precondition_compound(prec_fact, plan_action_key, prec, prea));
		}
	}

	return pre_comp;
}

void
ClipsExecutiveRestApi::gen_plan_precompute(PlanMap &plans, PlanActionMap &plan_actions,
                                           PreCompoundMap &prec, PreAtomMap &prea)
{
	CLIPS::Fact::pointer fact = clips_->get_facts();
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		if (tmpl->name() == "plan") {
			plans[std::make_pair(get_value<std::string>(fact, "goal-id"), get_value<std::string>(fact, "id"))] = fact;
		} else if (tmpl->name() == "plan-action") {
			plan_actions[std::make_pair(get_value<std::string>(fact, "goal-id"), get_value<std::string>(fact, "plan-id"))]
				.push_back(fact);
		} else if (tmpl->name() == "domain-precondition" || tmpl->name() == "domain-atomic-precondition") {
			std::string goal_id = get_value<std::string>(fact, "goal-id");
			std::string plan_id = get_value<std::string>(fact, "plan-id");
			int64_t action_id = get_value<int64_t>(fact, "grounded-with");
			if (action_id != 0) {
				if (tmpl->name() == "domain-precondition") {
					prec[std::make_tuple(goal_id, plan_id, action_id)].push_back(fact);
				} else {
					prea[std::make_tuple(goal_id, plan_id, action_id)].push_back(fact);
				}
			}
		}
		fact = fact->next();
	}
}

Plan
ClipsExecutiveRestApi::gen_plan(const PlanKey &plan_key,
                                const CLIPS::Fact::pointer fact,
                                PlanActionMap &plan_actions,
                                PreCompoundMap &prec, PreAtomMap &prea)
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

			int64_t action_id = get_value<int64_t>(pai, "id");
			std::string operator_name = get_value<std::string>(pai, "action-name");

			// general info
			pa->set_kind("PlanAction");
			pa->set_apiVersion(PlanAction::api_version());
			pa->set_id(action_id);
			pa->set_operator_name(operator_name);
			for (const auto& pv : get_values(pai, "param-values")) {
				pa->addto_param_values(std::move(pv));
			}
			pa->set_status(get_value<std::string>(pai, "status"));
			pa->set_executable(get_value<bool>(pai, "executable"));
			pa->set_duration(get_value<double>(pai, "duration"));
			pa->set_dispatch_time(get_value<double>(pai, "dispatch-time"));

			// preconditions
			const PlanActionKey plan_action_key{std::make_tuple(goal_id, plan_id, action_id)};
			if (prec.find(plan_action_key) != prec.end()) {
				for (auto& prec_fact : prec[plan_action_key]) {
					std::string part_of = get_value<std::string>(prec_fact, "part-of");
					int64_t grounded_with = get_value<int64_t>(prec_fact, "grounded-with");
					if (part_of == operator_name && grounded_with == action_id) {
						pa->addto_preconditions(gen_domain_precondition_compound(prec_fact, plan_action_key, prec, prea));
					}
				}
			}
			actions.push_back(std::move(pa));
		}

		std::sort(actions.begin(), actions.end(),
		          [](std::shared_ptr<PlanAction> &a, std::shared_ptr<PlanAction> &b)
		          {
			          return *a->id() < *b->id();
		          });
		p.set_actions(actions);
	}

	return p;
}


WebviewRestArray<Plan>
ClipsExecutiveRestApi::cb_list_plans()
{
	MutexLocker lock(clips_.objmutex_ptr());
	WebviewRestArray<Plan> rv;

	std::map<PlanKey, CLIPS::Fact::pointer> plans;
	std::map<PlanKey, ClipsFactList> plan_actions;
	PreCompoundMap prec;
	PreAtomMap prea;
	gen_plan_precompute(plans, plan_actions, prec, prea);
		
	for (auto &pi : plans) {
		rv.push_back(std::move(gen_plan(pi.first, pi.second, plan_actions, prec, prea)));
	}

	return rv;
}

Plan
ClipsExecutiveRestApi::cb_get_plan(WebviewRestParams& params)
{
	std::string goal_id = params.path_arg("goal-id");
	std::string id      = params.path_arg("id");

	MutexLocker lock(clips_.objmutex_ptr());
	WebviewRestArray<Plan> rv;

	std::map<PlanKey, CLIPS::Fact::pointer> plans;
	std::map<PlanKey, ClipsFactList> plan_actions;
	PreCompoundMap prec;
	PreAtomMap prea;

	gen_plan_precompute(plans, plan_actions, prec, prea);

	const PlanKey plan_key{goal_id, id};
	if (plans.find(plan_key) == plans.end()) {
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
		                           "No plan for goal '%s' with ID '%s' found",
		                           goal_id.c_str(), id.c_str());
	}

	return gen_plan(plan_key, plans[plan_key], plan_actions, prec, prea);
}
