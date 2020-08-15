/***************************************************************************
 *  pddl_ast.h - A static visitor to translate an effect
 *
 *  Created: Sun 22 Dec 2019 12:39:10 CET 12:39
 *  Copyright  2019 Daniel Habering <daniel.habering@rwth-aachen.de>
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

/*  This is based on pddl-qi (https://github.com/naderman/pddl-qi) */

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
	/**
	 * @brief Enum for all supported pddl requirements
	 *
	 */
	typedef enum { eStrips, eNegativePreconditions, typing, action_cost, adl } EnumType;

	/**
	 * @brief Collection of requirements needed in a specific PDDL domain
	 *
	 */
	typedef std::vector<EnumType> VectorType;
} RequirementFlag;

/**
 * @brief Operator flag struct
 *        Will be instanciated for each operator symbol
 *
 */
typedef struct OperatorFlag_
{
	/**
	 * @brief Enum for all possible logical operators
	 *
	 */
	typedef enum {

		negation,
		conjunction,
		disjunction,
		condition
	} EnumType;

	/**
	 * @brief Collection of logical operations
	 *
	 */
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
	/**
	 * @brief Name of the entity
	 *
	 */
	std::string name;

	/**
	 * @brief Type of the entity
	 *
	 */
	std::string type;

	/**
	 * @brief Construct a new Entity object
	 *
	 * @param n Name of the entity
	 * @param t Type of the entity
	 */
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
	/**
	 * @brief Bool denoting if the term is a variable or a constant
	 *
	 */
	bool isVariable;

	/**
	 * @brief Name of the term
	 *
	 */
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
	/**
	 * @brief name of the predicate of the atomic formula
	 *
	 */
	std::string predicateName;

	/**
	 * @brief List of terms that are the arguments of this atomic formula
	 *
	 */
	TermList args;
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
	/**
	 * @brief Boolean denoting if the literal is negated
	 *
	 */
	bool negate;

	/**
	 * @brief Atomic formula of the literal
	 *
	 */
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
	/**
	 * @brief Logical operator of the functional effect
	 *
	 */
	OperatorFlag::EnumType op;

	/**
	 * @brief Actual effect. Can be a list of Effects, a conditional effect or an atomic formula
	 *
	 */
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
	/**
	 * @brief Name of the variable that is influenced by the cost
	 *
	 */
	std::string name;

	/**
	 * @brief Cost of the cost
	 *
	 */
	int cost;
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
	/**
	 * @brief Condition that has to be true in order for the effect to be applied
	 *
	 */
	GoalDescription condition;

	/**
	 * @brief Effect to be applied if the condition is met
	 *
	 */
	Effect effect;
};

/**
 * @brief Struct representing a functional condition
 *        A functional condition is a list of conditions combined by an operator (eg. and, or)
 *
 */
struct FunctionalCondition
{
	/**
	 * @brief Logical operator of the functional condition (e.g. and, or)
	 *
	 */
	OperatorFlag::EnumType op;

	/**
	 * @brief A list of logical formulas connected by the logical operator
	 *
	 */
	std::vector<GoalDescription> condition;
};

/**
 * @brief Struct representing a PDDL action
 *
 */
struct PddlAction
{
	/**
	 * @brief Actionname
	 *
	 */
	std::string name;

	/**
	 * @brief Parameters of the action
	 *
	 */
	TypedList parameters;

	/**
	 * @brief Action precondition
	 *
	 */
	GoalDescription precondition;

	/**
	 * @brief Effect of the precondition
	 *
	 */
	Effect effect;

	/**
	 * @brief Breakup condition
	 *
	 */
	GoalDescription cond_breakup;

	/**
	 * @brief Temporal breakup condition
	 *
	 */
	GoalDescription temp_breakup;

	/**
	 * @brief Duration the action takes
	 *
	 */
	float duration;
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
	/**
	 * @brief Domain name
	 *
	 */
	std::string name;

	/**
	 * @brief Requirements of the domain
	 *
	 */
	RequirementFlag::VectorType requirements;

	/**
	 * @brief Available types in the domain
	 *
	 */
	TypedList types;

	/**
	 * @brief Constants of the domain
	 *
	 */
	TypedList constants;

	/**
	 * @brief Available predicates in the domain
	 *
	 */
	PredicateList predicates;

	/**
	 * @brief Available actions in the domain
	 *
	 */
	ActionList actions;
};

/**
 * @brief Struct representing a PDDL Problem
 *
 */
struct PddlProblem
{
	/**
	 * @brief Name of the problem
	 *
	 */
	std::string name;

	/**
	 * @brief Domain name this problem takes place
	 *
	 */
	std::string domain;

	/**
	 * @brief Currently present objects
	 *
	 */
	TypedList objects;

	/**
	 * @brief List of facts currently true
	 *
	 */
	FactList facts;

	/**
	 * @brief Goal condition that has to be met for a valid solution of this problem
	 *
	 */
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
