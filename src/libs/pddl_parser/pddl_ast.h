/**
 * This file is part of pddl_parser
 *
 * @copyright Nils Adermann <naderman@naderman.de>
 *
 * For the full copyright and licensing information please review the LICENSE
 * file that was distributed with this source code.
 */

#ifndef PDDLQI_PARSER_PDDLAST_H
#define PDDLQI_PARSER_PDDLAST_H

#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/qi_alternative.hpp>
#include <boost/variant/recursive_variant.hpp>
#include <string>
#include <vector>
using boost::variant;

namespace pddl_parser
{
    typedef struct RequirementFlag_ {
        typedef enum {
            eStrips,
            eNegativePreconditions,
            typing,
            action_cost,
            adl
        } EnumType;
        typedef std::vector<EnumType> VectorType;
    } RequirementFlag;

    struct Effect;
    struct ConditionalEffect;
    struct FunctionalCondition;

    struct Entity
    {
        std::string name;
        std::string type;
        Entity(const std::string n, const std::string t) : name(n), type(t) {}
    };

    typedef std::vector<struct Entity> TypedList;

    typedef std::vector<std::pair<std::string, TypedList> > PredicateList;

    typedef std::string Op;

    struct Term
    {
        bool isVariable;
        std::string name;
    };

    typedef std::vector<struct Term> TermList;

    struct AtomicFormula
    {
        std::string predicateName;
        TermList args;
    };

    struct Literal
    {
        bool negate;
        AtomicFormula atomicFormula;
    };

    struct FunctionalEffect
    {
      Op op;
      variant<boost::recursive_wrapper<std::vector<Effect>>,boost::recursive_wrapper<ConditionalEffect>> effect;
    };

    struct ActionCost
    {
        std::string name;
        int cost;
    };

    struct Effect
    {
        variant<FunctionalEffect,ActionCost,AtomicFormula> eff;
    };

    typedef boost::variant<boost::recursive_wrapper<FunctionalCondition>,AtomicFormula> GoalDescription;

    struct ConditionalEffect
    {
      GoalDescription condition;
      Effect effect;
    };

    struct FunctionalCondition
    {
      Op op;
      std::vector<GoalDescription> condition;
    };

    struct PddlAction
    {
        std::string name;
        TypedList parameters;
        GoalDescription precondition;
        Effect effect;
    };

    typedef std::vector<struct PddlAction> ActionList;

    struct PddlDomain
    {
        std::string name;
        RequirementFlag::VectorType requirements;
        TypedList types;
        TypedList constants;
        PredicateList predicates;
        ActionList actions;
    };
}

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::Entity,
    (std::string, name)
    (std::string, type)
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::Term,
    (bool, isVariable)
    (std::string, name)
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::AtomicFormula,
    (std::string, predicateName)
    (pddl_parser::TermList, args)
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::Literal,
    (bool, negate)
    (pddl_parser::AtomicFormula, atomicFormula)
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::FunctionalEffect,
    op,
    effect
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::ActionCost,
    name,
    cost
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::FunctionalCondition,
    op,
    condition
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::ConditionalEffect,
    condition,
    effect
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::Effect,
    eff
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::PddlAction,
    (std::string, name)
    (pddl_parser::TypedList, parameters)
    (pddl_parser::GoalDescription, precondition)
    (pddl_parser::Effect, effect)
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::PddlDomain,
    (std::string, name)
    (pddl_parser::RequirementFlag::VectorType, requirements)
    (pddl_parser::TypedList, types)
    (pddl_parser::TypedList, constants)
    (pddl_parser::PredicateList, predicates)
    (pddl_parser::ActionList, actions)
)

#endif
