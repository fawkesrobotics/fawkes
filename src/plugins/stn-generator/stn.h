
/***************************************************************************
 *  stn.h - stn-generator
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

#ifndef PLUGINS_STN_H_
#define PLUGINS_STN_H_

#include "boost/variant/static_visitor.hpp"
#include "domain_action.h"
#include "stn_action.h"

#include <aspect/logging.h>
#include <graphviz/gvc.h>
#include <pddl_parser/pddl_ast.h>

#include <algorithm>
#include <bsoncxx/document/value.hpp>
#include <iterator>
#include <mongocxx/client.hpp>
#include <string>
#include <vector>

namespace fawkes {
namespace stn {

class PreconditionVisitor : public boost::static_visitor<void>
{
public:
	PreconditionVisitor(std::vector<Predicate> *preconds, bool condition);

	void                    operator()(pddl_parser::AtomicFormula &a);
	void                    operator()(pddl_parser::FunctionalCondition &c);
	std::vector<Predicate> *preconds_;
	bool                    condition_;
};

class PddlStringPreconditionVisitor : public boost::static_visitor<void>
{
public:
	PddlStringPreconditionVisitor(std::ofstream &pddl_string);

	void           operator()(pddl_parser::AtomicFormula &a);
	void           operator()(pddl_parser::FunctionalCondition &c);
	std::ofstream &pddl_string_;
};

class BreakupVisitor : public boost::static_visitor<void>
{
public:
	BreakupVisitor(std::vector<std::string> *breakup);

	void                      operator()(pddl_parser::AtomicFormula &a);
	void                      operator()(pddl_parser::FunctionalCondition &c);
	std::vector<std::string> *breakup_;
};

class EffectVisitor : public boost::static_visitor<void>
{
public:
	EffectVisitor(std::vector<Predicate> *effects, bool condition);

	void                    operator()(pddl_parser::AtomicFormula &a);
	void                    operator()(pddl_parser::FunctionalEffect &fc);
	void                    operator()(pddl_parser::ActionCost &ac);
	std::vector<Predicate> *effects_;
	bool                    condition_;
};

class PddlStringEffectVisitor : public boost::static_visitor<void>
{
public:
	PddlStringEffectVisitor(std::ofstream &pddl_string);

	void           operator()(pddl_parser::AtomicFormula &a);
	void           operator()(pddl_parser::FunctionalEffect &fc);
	void           operator()(pddl_parser::ActionCost &ac);
	std::ofstream &pddl_string_;
};

class Stn
{
public:
	Stn(fawkes::Logger *logger);
	Stn(fawkes::Logger *logger, const std::string &classic_dom_path);
	virtual ~Stn();

	void add_plan_action(const std::string &name, const std::string &params);
	void set_initial_state(const StnAction &action);
	void read_initial_state(const std::string &pddl_problem_string);
	void set_pddl_domain(const std::string &pddl_domain_string);
	void generate();
	void drawGraph();
	std::vector<bsoncxx::document::value> get_bson();

private:
	struct plan_action
	{
		std::string name;
		std::string params;
	};

	fawkes::Logger *logger_;
	bool            gen_classic_dom_ = false;
	std::string     classic_dom_path_;
	StnAction       initial_state_;

	std::vector<DomainAction> domain_actions_;
	std::vector<plan_action>  plan_actions_;
	std::vector<StnAction>    stn_actions_;

	std::vector<std::pair<StnAction, StnAction>> cond_edges_;
	std::vector<std::pair<StnAction, StnAction>> temp_edges_;

	enum LogLevel { WARN, INFO, DEBUG };
	void      log_warn(const std::string &s);
	void      log_info(const std::string &s);
	void      log_debug(const std::string &s);
	void      log(const std::string &s, Stn::LogLevel log_leve);
	StnAction findActionById(size_t id);
	void      add_domain_action(const DomainAction &action);
	void      generate_classic_pddl_domain(pddl_parser::PddlDomain *dom,
	                                       const std::string &      classic_dom_path);
};

} // namespace stn
} // namespace fawkes

#endif
