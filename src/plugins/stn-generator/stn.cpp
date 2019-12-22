
/***************************************************************************
 *  stn.cpp - stn-generator
 *
 *  Created: Sat May  6 20:16:21 2017
 *  Copyright  2017  Matthias Loebach
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

#include "stn.h"

#include <pddl_parser/pddl_parser.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <exception>
#include <fstream>
#include <iostream>

namespace fawkes {
namespace stn {

/** @class Stn "stn.h"
 * A Simple Temporal Network.
 */

/** Constructor.
 * @param logger The logger to log to.
 */
Stn::Stn(fawkes::Logger *logger) : logger_(logger)
{
}

/** Constructor.
 * @param logger The logger to log to.
 * @param classic_dom_path The path to the domain file to write to.
 */
Stn::Stn(fawkes::Logger *logger, const std::string &classic_dom_path)
: logger_(logger), classic_dom_path_(classic_dom_path)
{
	gen_classic_dom_ = true;
}

/** Destructor */
Stn::~Stn()
{
}

/** Add the given DomainAction to the STN.
 * @param action The action to add.
 */
void
Stn::add_domain_action(const DomainAction &action)
{
	domain_actions_.push_back(action);
}

/** Add a (grounded action).
 * @param name The name of the action/operator.
 * @param params The parameters of the action.
 */
void
Stn::add_plan_action(const std::string &name, const std::string &params)
{
	plan_actions_.push_back(plan_action{name, params});
}

/** Set the initial state.
 * The resulting initial state is the state after applying the effects of the
 * given action.
 * @param action The action whose effects define the initial state.
 */
void
Stn::set_initial_state(const StnAction &action)
{
	initial_state_ = action;
}

/** Read the initial state from the given PDDL problem.
 * @param pddl_problem_string the PDDL rpboelm as (unparsed) string.
 */
void
Stn::read_initial_state(const std::string &pddl_problem_string)
{
	pddl_parser::Parser      parser;
	pddl_parser::PddlProblem prob = parser.parseProblem(pddl_problem_string);

	log_info("Parsing PDDL Problem for STN generation.");

	log_info("Parsed problem " + prob.name);
	std::vector<stn::Predicate> init_predicates;
	for (auto fact : prob.facts) {
		std::vector<std::string> attrs;
		std::string log_string = "Adding init-predicate " + fact.predicateName + " with arguments:";
		for (pddl_parser::Term a : fact.args) {
			attrs.push_back(a.name);
			log_string += " " + a.name;
		}
		log_info(log_string);
		stn::Predicate init_pred(boost::get<pddl_parser::AtomicFormula>(fact).predicateName,
		                         true,
		                         attrs);
		init_predicates.push_back(init_pred);
	}
	stn::StnAction init_action(prob.name, {}, init_predicates, std::string(""));
	set_initial_state(init_action);
}

/** Set the domain of the STN to the given PDDL domain.
 * This parses the given domain and processes all actions in the domain.
 * It also adds all temporal and conditional breakups defined in the domain to
 * the STN.
 * @param pddl_domain_string the PDDL domain as (unparsed) string.
 */
void
Stn::set_pddl_domain(const std::string &pddl_domain_string)
{
	pddl_parser::Parser     parser;
	pddl_parser::PddlDomain dom = parser.parseDomain(pddl_domain_string);

	log_info("Loading extended PDDL domain into STN.");

	for (auto &action : dom.actions) {
		log_info("Processing action " + action.name);
		std::vector<std::string> params;
		for (auto &param : action.parameters) {
			params.push_back(param.name);
		}
		std::vector<Predicate> preconds;
		PreconditionVisitor    precond_visitor(&preconds, true);
		boost::apply_visitor(precond_visitor, action.precondition);

		//		build_precon_list(action.precondition, &preconds, true);
		std::vector<Predicate> effects;
		//EffectVisitor eff_visitor(&effects,true);
		EffectVisitor eff_visitor(&effects, true);
		boost::apply_visitor(eff_visitor, action.effect.eff);
		//		build_eff_list(action.effect, &effects, true);

		int                      duration = action.duration;
		std::vector<std::string> cond_breakups;
		log_info(std::to_string(action.cond_breakup.which()));
		if (action.cond_breakup.which() == 1) { // only if type is Expression
			BreakupVisitor breakup_visitor(&cond_breakups);
			boost::apply_visitor(breakup_visitor, action.cond_breakup);
		}
		std::vector<std::string> temp_breakups;
		if (action.temp_breakup.which() == 1) { // only if type is Expression
			BreakupVisitor breakup_visitor(&temp_breakups);
			boost::apply_visitor(breakup_visitor, action.temp_breakup);
		}
		DomainAction da(action.name, params, preconds, effects, duration, cond_breakups, temp_breakups);
		domain_actions_.push_back(da);
	}

	log_info("Initialized " + std::to_string(domain_actions_.size()) + " domain actions");

	if (gen_classic_dom_) {
		log_info("Generation of classic domain file is configured, starting...");
		generate_classic_pddl_domain(&dom, classic_dom_path_);
	}
}

PreconditionVisitor::PreconditionVisitor(std::vector<Predicate> *preconds, bool condition)
: preconds_(preconds), condition_(condition)
{
}

void
PreconditionVisitor::operator()(pddl_parser::AtomicFormula &a)
{
	std::vector<std::string> args;
	for (pddl_parser::Term arg : a.args) {
		args.push_back(arg.name);
	}
	Predicate p(a.predicateName, condition_, args);
	this->preconds_->push_back(p);
}

void
PreconditionVisitor::operator()(pddl_parser::FunctionalCondition &c)
{
	if (c.op == pddl_parser::OperatorFlag_::conjunction
	    || c.op == pddl_parser::OperatorFlag_::negation) {
		if (c.op == pddl_parser::OperatorFlag_::negation) {
			condition_ = !condition_;
		}
		for (auto child : c.condition) {
			PreconditionVisitor precond_visitor(this->preconds_, this->condition_);
			boost::apply_visitor(precond_visitor, child);
		}
	}
}

EffectVisitor::EffectVisitor(std::vector<Predicate> *effects, bool condition)
: effects_(effects), condition_(condition)
{
}

void
EffectVisitor::operator()(pddl_parser::FunctionalEffect &fe)
{
	if (fe.op == pddl_parser::OperatorFlag_::conjunction
	    || fe.op == pddl_parser::OperatorFlag_::negation) {
		if (fe.op == pddl_parser::OperatorFlag_::negation) {
			condition_ = !condition_;
		}
		std::vector<pddl_parser::Effect> effects =
		  boost::get<std::vector<pddl_parser::Effect>>(fe.effect);
		for (auto eff : effects) {
			EffectVisitor eff_visitor(this->effects_, this->condition_);
			boost::apply_visitor(eff_visitor, eff.eff);
		}
	}
}

void
EffectVisitor::operator()(pddl_parser::AtomicFormula &a)
{
	std::vector<std::string> args;
	for (pddl_parser::Term arg : a.args) {
		args.push_back(arg.name);
	}
	Predicate p(a.predicateName, condition_, args);
	this->effects_->push_back(p);
}

void
EffectVisitor::operator()(pddl_parser::ActionCost &ac)
{
}

BreakupVisitor::BreakupVisitor(std::vector<std::string> *breakup) : breakup_(breakup)
{
}

void
BreakupVisitor::operator()(pddl_parser::AtomicFormula &a)
{
	this->breakup_->push_back(a.predicateName);
}

void
BreakupVisitor::operator()(pddl_parser::FunctionalCondition &c)
{
	if (c.op == pddl_parser::OperatorFlag_::conjunction
	    || c.op == pddl_parser::OperatorFlag_::negation) {
		for (auto child : c.condition) {
			BreakupVisitor breakup_visitor(this->breakup_);
			boost::apply_visitor(breakup_visitor, child);
		}
	}
}

/** Regenerate the STN. */
void
Stn::generate()
{
	stn_actions_.clear();
	//stn_actions_.push_back(initial_state_);

	for (plan_action pa : plan_actions_) {
		std::vector<DomainAction>::iterator it = domain_actions_.begin();
		for (; it != domain_actions_.end(); ++it) {
			if (it->getName() == pa.name) {
				break;
			}
		}
		if (it == domain_actions_.end())
			throw("could not find fitting DomainAction");
		DomainAction da = *(it);

		stn_actions_.push_back(da.generateStnAction(pa.name, pa.params));
	}
	std::cout << "Imported " << stn_actions_.size() << " actions into STN" << std::endl;

	for (int i = stn_actions_.size() - 1; i >= 0; i--) {
		std::vector<StnAction> candidate_actions =
		  std::vector<StnAction>(stn_actions_.begin(), stn_actions_.begin() + i);
		try {
			stn_actions_.at(i).genConditionalActions(candidate_actions);
		} catch (std::exception &e) {
			std::cout << "ERROR stn.cpp:" << e.what() << std::endl;
		}
	}

	std::vector<Predicate> predicates;
	for (std::vector<StnAction>::iterator it = stn_actions_.begin(); it != stn_actions_.end(); ++it) {
		// add conditional edges
		for (auto const &cond_action : it->condActionIds()) {
			std::pair<StnAction, StnAction> edge(findActionById(cond_action), findActionById(it->id()));
			cond_edges_.push_back(edge);
		}
		// add temporal edges
		bool break_edge = false;
		for (Predicate p : predicates) {
			if (it->checkForBreakup(EdgeType::TEMPORAL, p)) {
				break_edge = true;
				break;
			}
		}
		if (!break_edge && it != stn_actions_.begin()) {
			std::pair<StnAction, StnAction> edge(findActionById((it - 1)->id()),
			                                     findActionById(it->id()));
			temp_edges_.push_back(edge);
		}
		// handle predicates
		for (Predicate p : it->effects()) {
			if (p.condition()) {
				std::vector<Predicate>::iterator it = std::find(predicates.begin(), predicates.end(), p);
				if (it == predicates.end()) {
					predicates.push_back(p);
					//std::cout << "Added " << p;
				}
			} else {
				//std::cout << "Check for erase: " << p;
				Predicate                        neg_pred(p.name(), true, p.attrs());
				std::vector<Predicate>::iterator it =
				  std::find(predicates.begin(), predicates.end(), neg_pred);
				if (it != predicates.end()) {
					//std::cout << "Erased " << (*it);
					predicates.erase(it);
				}
			}
		}
	}
	//generate missing temporal links
	for (auto &a : stn_actions_) {
		bool no_temp_edge = true;
		for (auto &e : temp_edges_) {
			if (e.first == a) {
				no_temp_edge = false;
				break;
			}
		}
		if (no_temp_edge) {
			for (auto &ce : cond_edges_) {
				if (ce.first == a) {
					std::pair<StnAction, StnAction> edge(a, ce.second);
					temp_edges_.push_back(edge);
				}
			}
		}
	}
}

/** Render a graph representation of the STN.
 * This writes the graph representation to the file stn.png.
 */
void
Stn::drawGraph()
{
	Agraph_t *G;
	GVC_t *   gvc;

	gvc               = gvContext();
	char graph_name[] = "STN";

	G = agopen(graph_name, Agdirected, 0);

	std::map<size_t, Agnode_t *> node_map;

	for (StnAction a : stn_actions_) {
		std::string node_name = a.genGraphNodeName();
		node_map.insert(std::make_pair(a.id(), agnode(G, (char *)node_name.c_str(), true)));
	}

	std::vector<Agedge_t *> edge_list;
	for (auto &edge : cond_edges_) {
		Agnode_t *node1      = node_map.at(edge.first.id());
		Agnode_t *node2      = node_map.at(edge.second.id());
		Agedge_t *graph_edge = agedge(G, node1, node2, (char *)"conditional", true);
		edge_list.push_back(graph_edge);

		std::string edge_label = edge.second.genConditionEdgeLabel(edge.first.id());
		agsafeset(graph_edge,
		          (char *)"label",
		          agstrdup_html(G, (char *)edge_label.c_str()),
		          (char *)"");
		agsafeset(graph_edge, (char *)"color", (char *)"red", (char *)"");
	}

	for (auto &edge : temp_edges_) {
		Agnode_t *node1      = node_map.at(edge.first.id());
		Agnode_t *node2      = node_map.at(edge.second.id());
		Agedge_t *graph_edge = agedge(G, node1, node2, (char *)"temporal", true);
		edge_list.push_back(graph_edge);

		std::string edge_label = edge.second.genTemporalEdgeLabel();
		agsafeset(graph_edge,
		          (char *)"label",
		          agstrdup_html(G, (char *)edge_label.c_str()),
		          (char *)"");
		agsafeset(graph_edge, (char *)"color", (char *)"blue", (char *)"");
	}

	gvLayout(gvc, G, "dot");
	gvRenderFilename(gvc, G, "png", "stn.png");

	gvFreeLayout(gvc, G);
	agclose(G);
	gvFreeContext(gvc);
}

/** Get a BSON representation of the STN.
 * @return A vector of BSON objects, each element is an action.
 */
std::vector<bsoncxx::document::value>
Stn::get_bson()
{
	std::vector<bsoncxx::document::value> stn;
	for (auto &action : stn_actions_) {
		using namespace bsoncxx::builder;
		basic::document bson_action;
		bson_action.append(basic::kvp("id", static_cast<int64_t>(action.id())));
		bson_action.append(basic::kvp("name", action.name()));
		bson_action.append(basic::kvp("duration", static_cast<int64_t>(action.duration())));
		bson_action.append(basic::kvp("cond-actions", [action](basic::sub_array cond_actions) {
			for (auto &cond : action.condActionIds()) {
				cond_actions.append(static_cast<int64_t>(cond));
			}
		}));
		bson_action.append(basic::kvp("opts", [action](basic::sub_array opts) {
			std::stringstream                  opts_ss(action.opts());
			std::istream_iterator<std::string> end;
			for (std::istream_iterator<std::string> it(opts_ss); it != end; it++) {
				opts.append(*it);
			}
		}));
		stn.push_back(bson_action.extract());
	}
	return stn;
}

StnAction
Stn::findActionById(size_t id)
{
	for (StnAction a : stn_actions_) {
		if (a.id() == id) {
			return a;
		}
	}
	throw(" Action with id " + std::to_string(id) + " not found");
}

void
Stn::log_warn(const std::string &s)
{
	log(s, LogLevel::WARN);
}

void
Stn::log_info(const std::string &s)
{
	log(s, LogLevel::INFO);
}

void
Stn::log_debug(const std::string &s)
{
	log(s, LogLevel::DEBUG);
}
void
Stn::log(const std::string &s, Stn::LogLevel log_level)
{
	std::string name = "STN";
	switch (log_level) {
	case LogLevel::WARN: logger_->log_warn(name.c_str(), "%s", s.c_str()); break;
	case LogLevel::INFO: logger_->log_info(name.c_str(), "%s", s.c_str()); break;
	case LogLevel::DEBUG: logger_->log_debug(name.c_str(), "%s", s.c_str()); break;
	}
}

void
Stn::generate_classic_pddl_domain(pddl_parser::PddlDomain *dom, const std::string &classic_dom_path)
{
	log_info("Writing domain to " + classic_dom_path);
	std::ofstream out(classic_dom_path);

	out << "(define (domain " << dom->name << ")" << std::endl;

	out << "\t(:requirements";
	for (auto &req : dom->requirements) {
		out << " :" << req;
	}
	out << ")" << std::endl;

	out << "\t(:types" << std::endl;
	for (auto &type : dom->types) {
		out << "\t\t" << type.type << " - " << type.name << std::endl;
	}
	out << "\t)" << std::endl;

	out << "\t(:constants" << std::endl;
	for (auto &constant : dom->constants) {
		out << "\t\t";
		out << constant.type << " - " << constant.name << std::endl;
	}
	out << "\t)" << std::endl;

	out << "\t(:predicates" << std::endl;
	for (auto &predicate : dom->predicates) {
		out << "\t\t(" << predicate.first;
		for (auto &pred_type : predicate.second) {
			out << " ?" << pred_type.name << " - " << pred_type.type;
		}
		out << ")" << std::endl;
	}
	out << "\t)" << std::endl;

	for (auto &action : dom->actions) {
		out << "\t(:action " << action.name << std::endl;
		out << "\t\t:parameters (";
		for (auto &param : action.parameters) {
			out << " ?" << param.name << " - " << param.type;
		}
		out << ")" << std::endl;
		out << "\t\t:precondition" << std::endl << "\t\t\t";
		PddlStringPreconditionVisitor precond_visitor(out);
		boost::apply_visitor(precond_visitor, action.precondition);

		out << std::endl << "\t\t:effect" << std::endl << "\t\t\t";
		PddlStringEffectVisitor eff_visitor(out);
		boost::apply_visitor(eff_visitor, action.effect.eff);

		out << std::endl << "\t)" << std::endl;
	}

	out << ")";

	out.close();
}

PddlStringPreconditionVisitor::PddlStringPreconditionVisitor(std::ofstream &pddl_string)
: pddl_string_(pddl_string)
{
}

void
PddlStringPreconditionVisitor::operator()(pddl_parser::AtomicFormula &a)
{
	pddl_string_ << "(" << a.predicateName;
	for (pddl_parser::Term arg : a.args) {
		pddl_string_ << " " << arg.name;
	}
	pddl_string_ << ")";
}

void
PddlStringPreconditionVisitor::operator()(pddl_parser::FunctionalCondition &c)
{
	if (c.op == pddl_parser::OperatorFlag_::conjunction
	    || c.op == pddl_parser::OperatorFlag_::negation) {
		if (c.op == pddl_parser::OperatorFlag_::negation) {
			pddl_string_ << "(not ";
		} else if (c.op == pddl_parser::OperatorFlag_::conjunction) {
			pddl_string_ << "(and ";
		}
		for (auto child : c.condition) {
			PddlStringPreconditionVisitor precond_visitor(this->pddl_string_);
			boost::apply_visitor(precond_visitor, child);
		}
	}
}

PddlStringEffectVisitor::PddlStringEffectVisitor(std::ofstream &pddl_string)
: pddl_string_(pddl_string)
{
}

void
PddlStringEffectVisitor::operator()(pddl_parser::FunctionalEffect &fe)
{
	if (fe.op == pddl_parser::OperatorFlag_::conjunction
	    || fe.op == pddl_parser::OperatorFlag_::negation) {
		if (fe.op == pddl_parser::OperatorFlag_::negation) {
			pddl_string_ << "(not ";
		} else if (fe.op == pddl_parser::OperatorFlag_::conjunction) {
			pddl_string_ << "(and ";
		}
		std::vector<pddl_parser::Effect> effects =
		  boost::get<std::vector<pddl_parser::Effect>>(fe.effect);
		for (auto eff : effects) {
			PddlStringEffectVisitor eff_visitor(this->pddl_string_);
			boost::apply_visitor(eff_visitor, eff.eff);
		}
	}
}

void
PddlStringEffectVisitor::operator()(pddl_parser::AtomicFormula &a)
{
	pddl_string_ << "(" << a.predicateName;
	for (pddl_parser::Term arg : a.args) {
		pddl_string_ << " " << arg.name;
	}
	pddl_string_ << ")";
}

void
PddlStringEffectVisitor::operator()(pddl_parser::ActionCost &ac)
{
}

} // namespace stn
} // namespace fawkes
