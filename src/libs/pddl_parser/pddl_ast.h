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

/**
 * @brief Requirement flag struct. 
 *        Will be instanciated for each requirement of a PDDL Domain
 * 
 */
typedef struct RequirementFlag_
{
	typedef enum { eStrips, eNegativePreconditions, typing, action_cost, adl } EnumType;
	typedef std::vector<EnumType> VectorType;
} RequirementFlag;

/**
 * @brief Operator flag struct
 *        Will be instanciated for each operator symbol
 * 
 */
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

/**
 * @brief Struct representing an entity of a PDDL Domain.
 *        Contains name and type of the entity.
 * 
 */
struct Entity
{
	std::string name;
	std::string type;
	Entity(const std::string n, const std::string t) : name(n), type(t)
	{
	}
};

/**
 * @brief List of typed entities
 * 
 */
typedef std::vector<struct Entity> TypedList;

/**
 * @brief List of predicates
 *        A predicate contains of a name and a list of entities, representing the typed arguments
 * 
 */
typedef std::vector<std::pair<std::string, TypedList>> PredicateList;

/**
 * @brief Struct representing a term, which is either a variable or a constant
 * 
 */
struct Term
{
	bool        isVariable;
	std::string name;
};

/**
 * @brief List of terms
 * 
 */
typedef std::vector<struct Term> TermList;

/**
 * @brief Struct representing an atomic formula.
 *        An atomic formula consists of an instanciated predicate.
 *        Thus, containing a predicate name and a list of terms (either variables or constants)
 * 
 */
struct AtomicFormula
{
	std::string predicateName;
	TermList    args;
};

/**
 * @brief List of atomic formulas, representing a list of facts
 * 
 */
typedef std::vector<AtomicFormula> FactList;

/**
 * @brief Struct representing a literal
 * 
 */
struct Literal
{
	bool          negate;
	AtomicFormula atomicFormula;
};

/**
 * @brief Struct representing a functional effect
 *        A functional effect consists of an operator, which is succeeded either by a list of effects, 
 *        a conditional effect or a single atomic formula
 * 
 */
struct FunctionalEffect
{
	OperatorFlag::EnumType op;
	variant<boost::recursive_wrapper<std::vector<Effect>>,
	        boost::recursive_wrapper<ConditionalEffect>,
	        AtomicFormula>
	  effect;
};

/**
 * @brief Struct representing an action cost variable
 * 
 */
struct ActionCost
{
	std::string name;
	int         cost;
};

/**
 * @brief Struct representing an effect
 *        An effect can either be a functional effect, an atomic formula or an action cost
 * 
 */
struct Effect
{
	variant<FunctionalEffect, ActionCost, AtomicFormula> eff;
};

/**
 * @brief Definition of a goal description
 *        A goal description is either a single atomic formula or a functional condition
 * 
 */
using GoalDescription =
  boost::variant<boost::recursive_wrapper<FunctionalCondition>, AtomicFormula>;

/**
 * @brief A struct representing a conditional effect
 *        A conditional effect is an effect guarded by a condition. 
 *        Only if the condition is true, the effect is applied
 * 
 */
struct ConditionalEffect
{
	GoalDescription condition;
	Effect          effect;
};

/**
 * @brief Struct representing a functional condition
 *        A functional condition is a list of conditions combined by an operator (eg. and, or)
 * 
 */
struct FunctionalCondition
{
	OperatorFlag::EnumType       op;
	std::vector<GoalDescription> condition;
};

/**
 * @brief Struct representing a PDDL action
 * 
 */
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

/**
 * @brief A list of actions
 * 
 */
typedef std::vector<struct PddlAction> ActionList;

/**
 * @brief Struct representing a PDDL Domain
 * 
 */
struct PddlDomain
{
	std::string                 name;
	RequirementFlag::VectorType requirements;
	TypedList                   types;
	TypedList                   constants;
	PredicateList               predicates;
	ActionList                  actions;
};

/**
 * @brief Struct representing a PDDL Problem
 * 
 */
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
