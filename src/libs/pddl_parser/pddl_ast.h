/***************************************************************************
 *  stn.cpp - stn-generator
 *
 *  Created: Sun Dec 22 22:41 2019
 *  Copyright  Nils Adermann <naderman@naderman.de>, Daniel Habering
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

#ifndef PDDLQI_PARSER_PDDLAST_H
#define PDDLQI_PARSER_PDDLAST_H

#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/qi_alternative.hpp>
#include <boost/variant/recursive_variant.hpp>
#include <string>
#include <vector>
using boost::variant;

namespace pddl_parser {
typedef struct RequirementFlag_
{
	typedef enum { eStrips, eNegativePreconditions, typing, action_cost, adl } EnumType;
	typedef std::vector<EnumType> VectorType;
} RequirementFlag;

typedef struct OperatorFlag_
{
	typedef enum {

		negation,
		conjunction,
		disjunction,
		condition
	} EnumType;
	typedef std::vector<EnumType> VectorType;
} OperatorFlag;

struct Effect;
struct ConditionalEffect;
struct FunctionalCondition;

struct Entity
{
	std::string name;
	std::string type;
	Entity(const std::string n, const std::string t) : name(n), type(t)
	{
	}
};

typedef std::vector<struct Entity> TypedList;

typedef std::vector<std::pair<std::string, TypedList>> PredicateList;

struct Term
{
	bool        isVariable;
	std::string name;
};

typedef std::vector<struct Term> TermList;

struct AtomicFormula
{
	std::string predicateName;
	TermList    args;
};

typedef std::vector<AtomicFormula> FactList;

struct Literal
{
	bool          negate;
	AtomicFormula atomicFormula;
};

struct FunctionalEffect
{
	OperatorFlag::EnumType op;
	variant<boost::recursive_wrapper<std::vector<Effect>>,
	        boost::recursive_wrapper<ConditionalEffect>,
	        AtomicFormula>
	  effect;
};

struct ActionCost
{
	std::string name;
	int         cost;
};

struct Effect
{
	variant<FunctionalEffect, ActionCost, AtomicFormula> eff;
};

using GoalDescription =
  boost::variant<boost::recursive_wrapper<FunctionalCondition>, AtomicFormula>;

struct ConditionalEffect
{
	GoalDescription condition;
	Effect          effect;
};

struct FunctionalCondition
{
	OperatorFlag::EnumType       op;
	std::vector<GoalDescription> condition;
};

struct PddlAction
{
	std::string     name;
	TypedList       parameters;
	GoalDescription precondition;
	Effect          effect;
	GoalDescription cond_breakup;
	GoalDescription temp_breakup;
	float           duration;
};

typedef std::vector<struct PddlAction> ActionList;

struct PddlDomain
{
	std::string                 name;
	RequirementFlag::VectorType requirements;
	TypedList                   types;
	TypedList                   constants;
	PredicateList               predicates;
	ActionList                  actions;
};

struct PddlProblem
{
	std::string     name;
	std::string     domain;
	TypedList       objects;
	FactList        facts;
	GoalDescription goal;
};
} // namespace pddl_parser

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::Entity, (std::string, name)(std::string, type))

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::Term, (bool, isVariable)(std::string, name))

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::AtomicFormula,
                          (std::string, predicateName)(pddl_parser::TermList, args))

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::Literal,
                          (bool, negate)(pddl_parser::AtomicFormula, atomicFormula))

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::FunctionalEffect, op, effect)

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::ActionCost, name, cost)

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::FunctionalCondition,
                          (pddl_parser::OperatorFlag::EnumType,
                           op)(std::vector<pddl_parser::GoalDescription>, condition))

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::ConditionalEffect, condition, effect)

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::Effect, eff)

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::PddlAction,
                          (std::string, name)(pddl_parser::TypedList, parameters)(
                            pddl_parser::GoalDescription,
                            precondition)(pddl_parser::Effect,
                                          effect)(pddl_parser::GoalDescription,
                                                  cond_breakup)(pddl_parser::GoalDescription,
                                                                temp_breakup)(float, duration))

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::PddlDomain,
                          (std::string, name)(pddl_parser::RequirementFlag::VectorType,
                                              requirements)(pddl_parser::TypedList, types)(
                            pddl_parser::TypedList,
                            constants)(pddl_parser::PredicateList,
                                       predicates)(pddl_parser::ActionList, actions))

BOOST_FUSION_ADAPT_STRUCT(pddl_parser::PddlProblem,
                          (std::string, name)(std::string, domain)(pddl_parser::TypedList, objects)(
                            pddl_parser::FactList,
                            facts)(pddl_parser::GoalDescription, goal))

#endif
