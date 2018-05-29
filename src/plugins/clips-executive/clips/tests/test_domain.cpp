/***************************************************************************
 *  test_domain.cpp - Tests for the domain representation
 *
 *  Created: Tue 26 Sep 2017 11:05:41 CEST 11:05
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

#include "clips_test.h"

#include <vector>
#include <string>

#include <string>
#include <cstdio>

using namespace std;

/** Test setup for domain tests. */
class DomainTest : public CLIPSTest
{
  protected:
    /** Set up the test environment. */
    virtual void SetUp() {
      string logfile = tmpnam(nullptr);
      env.evaluate("(open \"" + logfile + "\" error \"w\")");
      LoadCLIPSFiles(clips_files);
      // Helps a lot to diagnose failures
      //env.evaluate("(watch facts)");
      //env.evaluate("(watch rules)");
    }
    /** These files are loaded during setup by default. */
	vector<string> clips_files =
	  { "../../../clips/clips/utils.clp", "../plan.clp", "../domain.clp" };
};

/** Test with the blocksworld domain. */
class BlocksworldDomainTest : public DomainTest
{
  protected:
    /** Set up the test environment. */
    virtual void SetUp() {
      clips_files.push_back("blocksworld.clp");
      DomainTest::SetUp();
    }
};

/** Check that a precondition of an action that should be executable is really
 * satisfied.
 */
TEST_F(BlocksworldDomainTest, PreconditionsAreSatisfiedTest)
{
  env.reset();
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-precondition))",
              "(and (eq ?p:name neg-on-table) (eq ?p:is-satisfied TRUE))"));
}

/** Check that a negative precondition that should not be satisfied is actually
 * not satisfied.
 */
TEST_F(BlocksworldDomainTest, NegativePreconditionIsNotSatisfied)
{
  env.reset();
  env.assert_fact("(domain-fact (name ontable) (param-values b1))");
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name neg-on-table) (eq ?p:is-satisfied FALSE))"));
  EXPECT_FALSE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name neg-on-table) (eq ?p:is-satisfied TRUE))"));
  EXPECT_TRUE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name pick-up-precond) (eq ?p:is-satisfied FALSE))"));
  EXPECT_FALSE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name pick-up-precond) (eq ?p:is-satisfied TRUE))"));
}

/** Test disjunctive preconditions. */
TEST_F(DomainTest, DisjunctivePreconditions)
{
  env.assert_fact("(domain-operator (name disjunctive-op))");
  env.assert_fact("(domain-precondition (name pre) (type disjunction)"
                  " (part-of disjunctive-op))");
  env.assert_fact("(plan-action (id 1) (goal-id g0) (plan-id p0) "
                  "(action-name disjunctive-op))");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:executable FALSE))"));
  env.assert_fact("(domain-predicate (name p))");
  env.assert_fact("(domain-atomic-precondition (part-of pre) (predicate p))");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:executable FALSE))"));
  env.assert_fact("(domain-fact (name p))");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:executable TRUE))"));
  env.assert_fact("(domain-predicate (name q))");
  env.assert_fact("(domain-atomic-precondition (part-of pre) (predicate q))");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:executable TRUE))"));
}

/** Check whether the function domain-is-precond-negative computes correct
 * values
 */
TEST_F(DomainTest, IsPrecondNegativeFunction)
{
  env.assert_fact("(domain-operator (name op))");
  env.assert_fact("(domain-precondition (name p1) (part-of op)"
                  "(type conjunction)"
                  ")");
  env.assert_fact("(domain-precondition (name p11) (part-of p1)"
                  "(type negation)"
                  ")");
  EXPECT_EQ("TRUE",
            env.evaluate("(domain-is-precond-negative p11)")[0].as_string());
  env.assert_fact("(domain-precondition (name p111) (part-of p11)"
                  "(type negation)"
                  ")");
  EXPECT_EQ("FALSE",
            env.evaluate("(domain-is-precond-negative p111)")[0].as_string());
  env.assert_fact("(domain-precondition (name p1111) (part-of p111)"
                  "(type disjunction)"
                  ")");
  EXPECT_EQ("FALSE",
            env.evaluate("(domain-is-precond-negative p1111)")[0].as_string());
  env.assert_fact("(domain-precondition (name p11111) (part-of p1111)"
                  "(type negation)"
                  ")");
  EXPECT_EQ("TRUE",
            env.evaluate("(domain-is-precond-negative p11111)")[0].as_string());
}

/** Ground an action with multiple parameters and check that the grounding is
 * correct. Also make sure its precondition is satisfied (which it should be).
 */
TEST_F(BlocksworldDomainTest, GroundingWithMultipleParameters)
{
  env.reset();
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-atomic-precondition))",
                       "(and (eq ?p:predicate on) "
                            "(eq ?p:grounded TRUE) "
                            "(eq ?p:param-values (create$ b1 b2)))"));
  EXPECT_TRUE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name unstack-precond) (eq ?p:is-satisfied TRUE))"));
}

/** Check whether some basic facts about types exist after adding some domain
 * objects.
 */
TEST_F(DomainTest, Typing)
{
  env.reset();
  env.assert_fact("(domain-object (name thing))");
  env.assert_fact("(domain-object-type (name moveable-obj))");
  env.assert_fact("(domain-object-type (name cup) (super-type moveable-obj))");
  env.assert_fact("(domain-object (name c1) (type cup))");
  env.run();
  EXPECT_TRUE(has_ordered_fact("domain-obj-is-of-type", { "thing", "object" }));
  EXPECT_TRUE(has_ordered_fact("domain-obj-is-of-type", { "c1", "cup" }));
  EXPECT_TRUE(has_ordered_fact("domain-obj-is-of-type", { "c1", "moveable-obj" }));
  EXPECT_TRUE(has_ordered_fact("domain-obj-is-of-type", { "c1", "object" }));
}

/** There should be no domain error if the specified domain is valid. */
TEST_F(BlocksworldDomainTest, NoErrorWithValidDomain)
{
  env.reset();
  env.run();
  EXPECT_FALSE(has_fact("((?error domain-error))"));
}

/** Every precondition must have an operator... */
TEST_F(DomainTest, ErrorIfPreconditionHasNoOperator)
{
  env.reset();
  env.assert_fact("(domain-precondition (name foo))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type precondition-without-parent)"));
}

/** ... and the operator must actually be defined. */
TEST_F(DomainTest, ErrorIfOperatorOfPreconditionDoesNotExist)
{
  env.reset();
  env.assert_fact("(domain-precondition (name foo) (part-of op))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type precondition-without-parent)"));
}

/** The type of an object must exist in the domain. */
TEST_F(DomainTest, ErrorIfObjTypeDoesNotExist)
{
  env.reset();
  env.assert_fact("(domain-object (name o1) (type t1))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type type-of-object-does-not-exist)"));
}

/** The super type of a domain type must exist. */
TEST_F(DomainTest, ErrorIfSuperTypeDoesNotExist)
{
  env.reset();
  env.assert_fact("(domain-object-type (name t2) (super-type t1))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type super-type-does-not-exist)"));
}

/** If the precondition of an action is satisfied, then the respective action
 * should also be marked as executable.
 */
TEST_F(BlocksworldDomainTest, ActionIsExecutableIfPreconditionIsSatisfied)
{
  env.reset();
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:action-name pick-up) (eq ?a:executable TRUE))"));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:action-name unstack) (eq ?a:executable TRUE))"));
}

/** Applying the effects of an action changes the domain facts. */
TEST_F(BlocksworldDomainTest, ApplyEffects)
{
  env.reset();
  env.run();
  EXPECT_FALSE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name holding) (eq ?p:param-values (create$ b1)))"));
  EXPECT_TRUE(has_fact("((?p domain-fact))", "(eq ?p:name handempty)"));
  EXPECT_TRUE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b1)))"));
  env.assert_fact("(apply-action g0 p0 1)");
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name holding) (eq ?p:param-values (create$ b1)))"));
  EXPECT_FALSE(has_fact("((?p domain-fact))", "(eq ?p:name handempty)"));
  EXPECT_FALSE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b1)))"));
}

/** When we apply one action after the other, then the effects of the latter
 * action must hold at the end. Also check that effects on the same predicate
 * but with different parameters are applied correctly.
 */
TEST_F(BlocksworldDomainTest, ApplyContradictingEffectsWithDifferentParams)
{
  env.reset();
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b1)))"));
  EXPECT_FALSE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b2)))"));
  env.assert_fact("(apply-action g0 p0 2)");
  env.run();
  EXPECT_FALSE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b1)))"));
  EXPECT_TRUE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b2)))"));
}

/** We wait for all sensed effects to occur before we apply any non-sensed
 * effects.
 */
TEST_F(DomainTest, WaitForSensedEffects)
{
  env.reset();
  env.assert_fact("(domain-operator (name drop))");
  env.assert_fact("(domain-operator-parameter"
                  " (operator drop)"
                  " (type object)"
                  " (name o)"
                  ")");
  env.assert_fact("(domain-predicate"
                  " (name holding)"
                  " (param-names o)"
                  " (sensed TRUE)"
                  ")");
  env.assert_fact("(domain-predicate"
                  " (name on-ground)"
                  " (param-names o)"
                  ")");
  env.assert_fact("(domain-effect"
                  " (type NEGATIVE)"
                  " (part-of drop) (predicate holding) (param-names o)"
                  ")");
  env.assert_fact("(domain-effect"
                  " (part-of drop) (predicate on-ground) (param-names o)"
                  ")");
  env.assert_fact("(domain-object (name obj1))");
  env.assert_fact("(domain-fact (name holding) (param-values obj1))");
  env.assert_fact("(plan-action"
                  " (goal-id g0) (plan-id p0)"
                  " (id 1)"
                  " (status EXECUTION-SUCCEEDED)"
                  " (action-name drop)"
                  " (param-names o)"
                  " (param-values obj1))");
  env.run();
  EXPECT_FALSE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:status FINAL))"));
  EXPECT_FALSE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:status SENSED-EFFECTS-HOLD))"));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:status SENSED-EFFECTS-WAIT))"));
  EXPECT_FALSE(has_fact("((?f domain-fact))",
        "(and (eq ?f:name on-ground) (eq ?f:param-values (create$ obj1)))"));
  env.evaluate("(delayed-do-for-all-facts ((?df domain-fact)) "
               " (and (eq ?df:name holding) (eq ?df:param-values (create$ obj1))) "
               "   (retract ?df)"
               ")");
  env.run();
  EXPECT_TRUE(has_fact("((?f domain-fact))",
        "(and (eq ?f:name on-ground) (eq ?f:param-values (create$ obj1)))"));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:status FINAL))"));
}

/** Do not wait if the operator has wait-sensed set to FALSE. */
TEST_F(DomainTest, OnlyWaitForEffectsIfWaitSensedIsTRUE)
{
  env.reset();
  env.assert_fact("(domain-operator (name drop) (wait-sensed FALSE))");
  env.assert_fact("(domain-operator-parameter"
                  " (operator drop)"
                  " (type object)"
                  " (name o)"
                  ")");
  env.assert_fact("(domain-predicate"
                  " (name holding)"
                  " (param-names o)"
                  " (sensed TRUE)"
                  ")");
  env.assert_fact("(domain-predicate"
                  " (name on-ground)"
                  " (param-names o)"
                  ")");
  env.assert_fact("(domain-effect"
                  " (type NEGATIVE)"
                  " (part-of drop) (predicate holding) (param-names o)"
                  ")");
  env.assert_fact("(domain-effect"
                  " (part-of drop) (predicate on-ground) (param-names o)"
                  ")");
  env.assert_fact("(domain-object (name obj1))");
  env.assert_fact("(domain-fact (name holding) (param-values obj1))");
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (goal-id g0) (plan-id p0)"
                  " (status EXECUTION-SUCCEEDED)"
                  " (action-name drop)"
                  " (param-names o)"
                  " (param-values obj1))");
  env.run();
  EXPECT_TRUE(has_fact("((?f domain-fact))",
        "(and (eq ?f:name on-ground) (eq ?f:param-values (create$ obj1)))"));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:status FINAL))"));
}

/** Sensed effects of an exogenous action are dropped from the precondition. */
TEST_F(DomainTest, ExogenousActions)
{
  env.reset();
  env.assert_fact("(domain-predicate"
                  " (name holding)"
                  " (param-names o)"
                  " (sensed TRUE)"
                  ")");
  env.assert_fact("(domain-operator (name drop) (exogenous TRUE))");
  env.assert_fact("(domain-operator-parameter"
                  " (operator drop)"
                  " (type object)"
                  " (name o)"
                  ")");
  env.assert_fact("(domain-precondition"
                  " (name drop-cond)"
                  " (part-of drop)"
                  ")");
  env.assert_fact("(domain-atomic-precondition (part-of drop-cond) "
                  " (predicate holding)"
                  " (param-names o)"
                  ")");
  env.assert_fact("(domain-effect"
                  " (type NEGATIVE)"
                  " (part-of drop) (predicate holding) (param-names o)"
                  ")");
  env.assert_fact("(domain-object (name obj1))");
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (goal-id g0) (plan-id p0)"
                  " (action-name drop)"
                  " (param-names o)"
                  " (param-values obj1))");
  env.run();
  // The precondition (holding obj1) is false, but since it is a sensed effect,
  // it should be removed from the operator's precondition.
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:executable TRUE))"));
}

/** If an action has the same effect both as positive and as negative effect,
 * then the effect should hold after the effects are applied.
 */
TEST_F(DomainTest, ApplyCondtradictingEffectsWithSameParams)
{
  env.reset();
  env.assert_fact("(domain-operator (name op1))");
  env.assert_fact("(domain-operator-parameter (name x) (operator op1))");
  env.assert_fact("(domain-predicate (name p) (param-names x))");
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (status EXECUTION-SUCCEEDED)"
                  " (action-name op1)"
                  " (param-names x)"
                  " (param-values a))");
  env.assert_fact("(domain-effect"
                  " (part-of op1)"
                  " (predicate p)"
                  " (type NEGATIVE)"
                  " (param-names x)"
                  ")");
  env.assert_fact("(domain-effect"
                  " (part-of op1)"
                  " (predicate p)"
                  " (type POSITIVE)"
                  " (param-names x)"
                  ")");
  env.assert_fact("(domain-effect"
                  " (part-of op1)"
                  " (predicate p)"
                  " (type NEGATIVE)"
                  " (param-names x)"
                  ")");
  env.run();
  EXPECT_TRUE(has_fact("((?f domain-fact))",
        "(and (eq ?f:name p) (eq ?f:param-values (create$ a)))"));
}

/** Test whether constants in preconditions work as expected. */
TEST_F(DomainTest, PreconditionWithConstant)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (goal-id g0) (plan-id p0)"
                  " (action-name op1)"
                  " (param-names y)"
                  " (param-values b))");
  env.assert_fact("(domain-precondition (name p1) (part-of op1))");
  env.assert_fact("(domain-atomic-precondition"
             " (name ap1)"
             " (part-of p1)"
             " (predicate pred1)"
             " (param-names c y)"
             " (param-constants a))");
  env.run();
  EXPECT_FALSE(has_fact("((?p domain-atomic-precondition))",
        "(and (eq ?p:name ap1) (eq ?p:is-satisfied TRUE))"));
  EXPECT_FALSE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name p1) (eq ?p:is-satisfied TRUE))"));
  EXPECT_FALSE(has_fact("((?a plan-action))",
        "(and (eq a:?id 1) (eq ?a:executable TRUE))"));
  env.assert_fact("(domain-fact (name pred1) (param-values a b))");
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-atomic-precondition))",
        "(and (eq ?p:name ap1) (eq ?p:is-satisfied TRUE))"));
  EXPECT_TRUE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name p1) (eq ?p:is-satisfied TRUE))"));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:executable TRUE))"));
}

/** Test whether the order of parameters and constants is correct if the
 * constant is the second parameter of the precondition.
 */
TEST_F(DomainTest, PreconditionWithConstantInSecondSlot)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (goal-id g0) (plan-id p0)"
                  " (action-name op1)"
                  " (param-names x)"
                  " (param-values b))");
  env.assert_fact("(domain-precondition (name p1) (part-of op1))");
  env.assert_fact("(domain-atomic-precondition"
             " (name ap1)"
             " (part-of p1)"
             " (predicate pred1)"
             " (param-names x c)"
             " (param-constants nil a))");
  env.run();
  EXPECT_FALSE(has_fact("((?p domain-atomic-precondition))",
        "(and (eq ?p:name ap1) (eq ?p:is-satisfied TRUE))"));
  EXPECT_FALSE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name p1) (eq ?p:is-satisfied TRUE))"));
  EXPECT_FALSE(has_fact("((?a plan-action))",
        "(and (eq a:?id 1) (eq ?a:executable TRUE))"));
  env.assert_fact("(domain-fact (name pred1) (param-values b a))");
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-atomic-precondition))",
        "(and (eq ?p:name ap1) (eq ?p:is-satisfied TRUE))"));
  EXPECT_TRUE(has_fact("((?p domain-precondition))",
        "(and (eq ?p:name p1) (eq ?p:is-satisfied TRUE))"));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:executable TRUE))"));
}

/** Test equality predicates. */
TEST_F(DomainTest, Equality)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (status PENDING)"
                  " (goal-id g0) (plan-id p0)"
                  " (action-name op1)"
                  " (param-names x y)"
                  " (param-values b b))");
  env.assert_fact("(domain-precondition (name p1) (part-of op1))");
  env.assert_fact("(domain-atomic-precondition"
             " (name ap1)"
             " (part-of p1)"
             " (equality TRUE)"
             " (param-names x y)"
             ")");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:executable TRUE))"));
  env.assert_fact("(plan-action"
                  " (id 2)"
                  " (status PENDING)"
                  " (goal-id g0) (plan-id p0)"
                  " (action-name op1)"
                  " (param-names x y)"
                  " (param-values b c))");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 2) (eq ?a:executable FALSE))"));
}

/** Test that errors of equality conditions are properly detected.  */
TEST_F(DomainTest, EqualityErrors)
{
  env.reset();
  env.assert_fact("(domain-atomic-precondition"
             " (name ap1)"
             " (part-of p1)"
             " (equality TRUE)"
             " (param-names x)"
             ")");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type equality-must-have-exactly-two-parameters)"));
  env.reset();
  env.assert_fact("(domain-atomic-precondition"
             " (name ap1)"
             " (part-of p1)"
             " (equality TRUE)"
             " (param-names x y z)"
             ")");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type equality-must-have-exactly-two-parameters)"));
  env.reset();
  env.assert_fact("(domain-atomic-precondition"
             " (name ap1)"
             " (part-of p1)"
             " (equality TRUE)"
             " (predicate p)"
             " (param-names x y)"
             ")");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type precondition-with-equality-and-predicate)"));
  env.reset();
  env.assert_fact("(domain-atomic-precondition"
             " (name ap1)"
             " (part-of p1)"
             " (param-names x y)"
             ")");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type precondition-must-have-predicate-or-be-equality)"));
}


/** If there is an unknown parameter in a precondition, then the domain contains
 * an error.
 */
TEST_F(DomainTest, PreconditionWithUnknownParameter)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (goal-id g0) (plan-id p0)"
                  " (action-name op1)"
                  " (param-names x)"
                  " (param-values b))");
  env.assert_fact("(domain-precondition (name p1) (part-of op1))");
  env.assert_fact("(domain-atomic-precondition"
             " (name ap1)"
             " (part-of p1)"
             " (predicate pred1)"
             " (param-names y c)"
             " (param-constants nil a))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type unknown-parameter)"));
}

/** Each action has an operator that is defined in the domain. */
TEST_F(DomainTest, ActionHasADomainOperator)
{
  env.reset();
  env.assert_fact("(plan-action (action-name doesn-not-exist))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type operator-of-action-does-not-exist)"));
}

/** Value predicates must take at most one value. */
TEST_F(DomainTest, ValuePredicatesHaveUniqueValues)
{
  env.reset();
  env.assert_fact("(domain-predicate (name p) (value-predicate TRUE))");
  env.assert_fact("(domain-fact (name p) (param-values a b c 1))");
  env.assert_fact("(domain-fact (name p) (param-values a b d 1))");
  env.run();
  EXPECT_FALSE(has_fact("((?e domain-error))",
        "(eq ?e:error-type value-predicate-with-multiple-values)"));
  env.assert_fact("(domain-fact (name p) (param-values a b c 2))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type value-predicate-with-multiple-values)"));
  env.evaluate("(do-for-fact ((?f domain-fact)) "
      "(and (eq ?f:name p) (eq ?f:param-values (create$ a b c 2)))"
      "(retract ?f)"
      ")");
  env.run();
  EXPECT_FALSE(has_fact("((?e domain-error))",
        "(eq ?e:error-type value-predicate-with-multiple-values)"));
}

TEST_F(BlocksworldDomainTest, EffectsOnValuePredicatesMustOccurInPairs)
{
  env.reset();
  env.run();
  EXPECT_FALSE(has_fact("((?e domain-error))",
        "(eq ?e:error-type value-predicate-without-paired-effect)"));
  env.evaluate("(do-for-fact ((?p domain-predicate)) "
               "(eq ?p:name holding) "
               "(duplicate ?p (value-predicate TRUE))"
               "(retract ?p)"
               ")");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))",
        "(eq ?e:error-type value-predicate-without-paired-effect)"));
}


/** Action IDs are only unique within the same plan and goal.
 *  Create multiple actions with the same ID but different plans, and test
 *  if they are treated correctly.
 */
TEST_F(BlocksworldDomainTest, NonUniqueActionIDs)
{
  env.reset();
  env.assert_fact("(plan-action (id 1) (goal-id g0) (plan-id p1)"
                  " (action-name stack)"
                  " (param-names x y) (param-values b1 b2))");
  env.assert_fact("(plan-action (id 1) (goal-id g1) (plan-id p0)"
                  " (action-name stack)"
                  " (param-names x y) (param-values b1 b2))");
  env.run();
  // The action pick-up with ID 1 in plan p0 of goal g0. This should be
  // executable, and the executable state should not be copied to the other
  // actions.
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:goal-id g0) (eq ?a:plan-id p0)"
        " (eq ?a:executable TRUE))"
        ));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:goal-id g0) (eq ?a:plan-id p1)"
        " (eq ?a:executable FALSE))"
        ));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:goal-id g0) (eq ?a:plan-id p0)"
        " (eq ?a:executable TRUE))"
        ));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:id 1) (eq ?a:goal-id g1) (eq ?a:plan-id p0)"
        " (eq ?a:executable FALSE))"
        ));
}

/** Test with the conditional-say domain. */
class ConditionalSayDomainTest : public DomainTest
{
  protected:
    /** Set up the test environment. */
    virtual void SetUp() {
      clips_files.push_back("conditional_say.clp");
      DomainTest::SetUp();
    }
};

/** A conditional effect is not applied if the condition does not hold. */
TEST_F(ConditionalSayDomainTest, DISABLED_DoNotApplyCondEffectIfCondDoesNotHold)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (goal-id g0) (plan-id p0)"
                  " (action-name say)"
                  " (param-names s t)"
                  " (param-values front_speaker hello)"
                  ")");
  env.assert_fact("(apply-action g0 p0 1)");
  env.run();
  EXPECT_FALSE(has_fact("((?fact domain-fact))",
        "(and (eq ?fact:name said) (eq ?fact:param-values (create$ hello)))"));
}

/** A conditional effect is applied if the condition holds. */
TEST_F(ConditionalSayDomainTest, ApplyCondEffectIfCondHolds)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
                  " (goal-id g0) (plan-id p0)"
                  " (action-name say)"
                  " (param-names s t)"
                  " (param-values front_speaker hello)"
                  ")");
  env.assert_fact("(apply-action g0 p0 1)");
  env.assert_fact(
      "(domain-fact (name speaker-ready) (param-values front_speaker))");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))", "(eq ?a:status FINAL)"));
  EXPECT_TRUE(has_fact("((?fact domain-fact))",
        "(and (eq ?fact:name said) (eq ?fact:param-values (create$ hello)))"));
}

/** A precondition of an exogenous action on a non-value predicate should be
 * removed if the action has the same predicate with the same parameters as
 * effect.
 */
TEST_F(DomainTest, ExogenousActionWithNonValuePredicatePrecondition)
{
  env.reset();
  env.assert_fact("(domain-operator (name put) (exogenous TRUE)"
      " (param-names mps wp)" ")");
  env.assert_fact("(domain-operator-parameter (name mps) (operator put)"
      "(type object)" ")");
  env.assert_fact("(domain-operator-parameter (name wp) (operator put)"
      "(type object)" ")");
  env.assert_fact("(domain-predicate (name wp-at) (sensed TRUE)"
      " (param-names mps wp) (param-types object object)" ")");
  env.assert_fact("(domain-precondition (operator put) (part-of put)"
      " (name put-precond) (type conjunction)" ")");
  env.assert_fact("(domain-precondition (operator put) (part-of put-precond)"
      " (name put-precond1) (type negation)" ")");
  env.assert_fact("(domain-atomic-precondition (operator put)"
      " (name put-precond11)"
      " (part-of put-precond1) (predicate wp-at) (param-names mps wp)" ")");
  env.assert_fact("(domain-effect (part-of put) (predicate wp-at)"
      " (param-names mps wp)" ")");
  env.assert_fact("(plan-action (id 1) (goal-id g0) (plan-id p0)"
      " (status FORMULATED)"
      " (action-name put) (param-names mps wp) (param-values C-CS wp1)" ")");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))", "(eq ?a:executable TRUE)"));
}

/** A precondition of an exogenous action on a value predicate should be
 * replaced by a disjunction of the original precondition and the effect of the
 * action if the action changes the value of the value predicate.
 */
TEST_F(DomainTest, ExogenousActionWithValuePredicatePrecondition)
{
  env.reset();
  env.assert_fact("(domain-operator (name dispense) (exogenous TRUE)"
      " (param-names mps)" ")");
  env.assert_fact("(domain-operator-parameter (name mps) (operator dispense)"
      "(type object)" ")");
  env.assert_fact("(domain-predicate (name mps-state) (sensed TRUE)"
      " (param-names mps state) (param-types object object)"
      " (value-predicate TRUE)" ")");
  env.assert_fact("(domain-precondition (operator dispense) (part-of dispense)"
      " (name dispense-precond) (type conjunction)" ")");
  env.assert_fact("(domain-atomic-precondition (operator dispense)"
      " (part-of dispense-precond) (predicate mps-state) (param-names mps c)"
      " (param-constants nil PROCESSING)" ")");
  env.assert_fact("(domain-effect (part-of dispense) (predicate mps-state)"
      " (param-names mps c) (param-constants nil READY-AT-OUTPUT)" ")");
  env.assert_fact("(domain-effect (part-of dispense) (predicate mps-state)"
      " (type NEGATIVE)"
      " (param-names mps c) (param-constants nil PROCESSING)" ")");
  env.assert_fact("(plan-action (id 1) (goal-id g0) (plan-id p0)"
      " (status FORMULATED)"
      " (action-name dispense) (param-names mps) (param-values C-CS)" ")");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))", "(eq ?a:executable FALSE)"));
  env.assert_fact("(domain-fact (name mps-state)"
     " (param-values C-CS READY-AT-OUTPUT)" ")");
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))", "(eq ?a:executable TRUE)"));
}
