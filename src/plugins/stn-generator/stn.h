
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

/**
 * @brief Visitor class to access Boost spirit Precondition objects
 *        Parses the predicates of the precondition into a vector of predicates
 * 
 */
class PreconditionVisitor : public boost::static_visitor<void>
{
public:
	/**
	 * @brief Construct a new Precondition Visitor object
	 * 
	 * @param preconds pointer to a vector of Predicate objects, to be filled with the predicates of the precondition
	 * @param condition Bool for denoting if the precondition is a negation
	 */
	PreconditionVisitor(std::vector<Predicate> *preconds, bool condition);

	/**
	 * @brief Handler if the precondition is an atomic formula
	 * 
	 * @param a pddl_parser AtomicFormula struct
	 */
	void operator()(pddl_parser::AtomicFormula &a);

	/**
	 * @brief Handler if the precondition is a functional condition
	 * 
	 * @param c pddl_parser FunctionalCondition struct
	 */
	void operator()(pddl_parser::FunctionalCondition &c);

	/**
	 * @brief Pointer to a vector of predicates, to be filled by the handler functions
	 * 
	 */
	std::vector<Predicate> *preconds_;

	/**
	 * @brief Boolean denoting the current negation status
	 * 
	 */
	bool condition_;
};

/**
 * @brief Visitor class to access Boost spirit Precondition objects
 *        Parses the predicates of the precondition into a ofstream, to be later used as string
 * 
 */
class PddlStringPreconditionVisitor : public boost::static_visitor<void>
{
public:
	/**
	 * @brief Construct a new Pddl String Precondition Visitor object
	 * 
	 * @param pddl_string ofstream to collect the predicates of the precondition
	 */
	PddlStringPreconditionVisitor(std::ofstream &pddl_string);

	/**
	 * @brief Handler if the precondition is an atomic formula
	 * 
	 * @param a pddl_parser AtomicFormula struct
	 */
	void operator()(pddl_parser::AtomicFormula &a);

	/**
	 * @brief Handler if the precondition is a functional condition
	 * 
	 * @param c pddl_parser FunctionalCondition struct
	 */
	void operator()(pddl_parser::FunctionalCondition &c);

	/**
	 * @brief ofstream to store the parsed predicates of the precondition as string
	 * 
	 */
	std::ofstream &pddl_string_;
};

/**
 * @brief Visitor class to access Boost spirit Breakup objects
 *        Parses the predicates of the breakup into a vector of strings
 * 
 */
class BreakupVisitor : public boost::static_visitor<void>
{
public:
	/**
	 * @brief Construct a new Breakup Visitor object
	 * 
	 * @param breakup pointer to a vector of strings the breakup predicates are parsed into
	 */
	BreakupVisitor(std::vector<std::string> *breakup);

	/**
	 * @brief Handler if the Breakup is an atomic formula
	 * 
	 * @param a pddl_parser AtomicFormula struct
	 */
	void operator()(pddl_parser::AtomicFormula &a);

	/**
	 * @brief Handler if the Breakup is a functional condition
	 * 
	 * @param c pddl_parser FunctionCondition struct
	 */
	void operator()(pddl_parser::FunctionalCondition &c);

	/**
	 * @brief pointer to a vector of strings, to store the parsed predicates
	 * 
	 */
	std::vector<std::string> *breakup_;
};

/**
 * @brief Visitor class to access Boost spirit Effect objects
 *        Parses the predicates of the effect into a vector of predicates
 * 
 */
class EffectVisitor : public boost::static_visitor<void>
{
public:
	/**
	 * @brief Construct a new Effect Visitor object
	 * 
	 * @param effects Pointer to a vector of predicates where the predicates of the effect are parsed into
	 * @param condition Bool for denoting if the predicate is a negation
	 */
	EffectVisitor(std::vector<Predicate> *effects, bool condition);

	/**
	 * @brief Handler if the effect is an atomic formula
	 * 
	 * @param a pddl_parser AtomicFormula struct
	 */
	void operator()(pddl_parser::AtomicFormula &a);

	/**
	 * @brief Handler if the effect is a functional effect (e.g. (and A B) )
	 * 
	 * @param fc pddl_parser FunctionalEffect struct
	 */
	void operator()(pddl_parser::FunctionalEffect &fc);

	/**
	 * @brief Handler if the visited effect is the action cost
	 * 
	 * @param ac pddl_parser ActionCost struct
	 */
	void operator()(pddl_parser::ActionCost &ac);

	/**
	 * @brief pointer to a vector of predicates. Will be filled by the handler functions
	 * 
	 */
	std::vector<Predicate> *effects_;

	/**
	 * @brief Boolean to denote the current negation status of the visited effect
	 * 
	 */
	bool condition_;
};

/**
 * @brief Visitor class to access Boost spirit Effect objects
 *        Parses the predicates of the effect into a vector of strings
 * 
 */
class PddlStringEffectVisitor : public boost::static_visitor<void>
{
public:
	/**
	 * @brief Construct a new Pddl String Effect Visitor object
	 * 
	 * @param pddl_string stringstream to collect the predicates of the effect as string
	 */
	PddlStringEffectVisitor(std::ofstream &pddl_string);

	/**
	 * @brief Handler if the effect is an atomic formula
	 * 
	 * @param a pddl_parser AtomicFormula struct
	 */
	void operator()(pddl_parser::AtomicFormula &a);

	/**
	 * @brief Handler if the effect is a functional effect (e.g. (and A B) )
	 * 
	 * @param fc pddl_parser FunctionalEffect struct
	 */
	void operator()(pddl_parser::FunctionalEffect &fc);

	/**
	 * @brief Handler if the visited effect is the action cost
	 * 
	 * @param ac pddl_parser ActionCost struct
	 */
	void operator()(pddl_parser::ActionCost &ac);

	/**
	 * @brief ofstream to store the parsed predicates as string
	 * 
	 */
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
