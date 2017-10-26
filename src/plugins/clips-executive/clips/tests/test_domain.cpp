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

using namespace std;

class DomainTest : public CLIPSTest
{
  protected:
    virtual void SetUp() {
      LoadCLIPSFiles(clips_files);
    }
    vector<string> clips_files = { "../plan.clp", "../domain.clp" };
};

class BlocksworldDomainTest : public DomainTest
{
  protected:
    virtual void SetUp() {
      clips_files.push_back("blocksworld.clp");
      DomainTest::SetUp();
    }
};

TEST_F(BlocksworldDomainTest, PreconditionsAreSatisfiedTest)
{
  env.reset();
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-precondition))",
              "(and (eq ?p:name neg-on-table) (eq ?p:is-satisfied TRUE))"));
}

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

TEST_F(DomainTest, Typing)
{
  env.reset();
  env.assert_fact("(domain-object (name thing))");
  env.assert_fact("(domain-object-type (name moveable-obj))");
  env.assert_fact("(domain-object-type (name cup) (super-type moveable-obj))");
  env.assert_fact("(domain-object (name c1) (type cup))");
  env.run();
  EXPECT_TRUE(has_ordered_fact("obj-is-of-type", { "thing", "object" }));
  EXPECT_TRUE(has_ordered_fact("obj-is-of-type", { "c1", "cup" }));
  EXPECT_TRUE(has_ordered_fact("obj-is-of-type", { "c1", "moveable-obj" }));
  EXPECT_TRUE(has_ordered_fact("obj-is-of-type", { "c1", "object" }));
}

TEST_F(BlocksworldDomainTest, NoErrorWithValidDomain)
{
  env.reset();
  env.run();
  EXPECT_FALSE(has_fact("((?error domain-error))"));
}

TEST_F(DomainTest, ErrorIfPreconditionHasNoOperator)
{
  env.reset();
  env.assert_fact("(domain-precondition (name foo))");
  env.run();
  EXPECT_TRUE(has_fact("((?error domain-error))"));
}

TEST_F(DomainTest, ErrorIfOperatorOfPreconditionDoesNotExist)
{
  env.reset();
  env.assert_fact("(domain-precondition (name foo) (part-of op))");
  env.run();
  EXPECT_TRUE(has_fact("((?error domain-error))"));
}

TEST_F(DomainTest, ErrorIfObjTypeDoesNotExist)
{
  env.reset();
  env.assert_fact("(domain-object (name o1) (type t1))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))"));
}

TEST_F(DomainTest, ErrorIfSuperTypeDoesNotExist)
{
  env.reset();
  env.assert_fact("(domain-object-type (name t2) (super-type t1))");
  env.run();
  EXPECT_TRUE(has_fact("((?e domain-error))"));
}

TEST_F(BlocksworldDomainTest, ActionIsExecutableIfPreconditionIsSatisfied)
{
  env.reset();
  env.run();
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:action-name pick-up) (eq ?a:executable TRUE))"));
  EXPECT_TRUE(has_fact("((?a plan-action))",
        "(and (eq ?a:action-name unstack) (eq ?a:executable TRUE))"));
}

TEST_F(BlocksworldDomainTest, ApplyEffects)
{
  env.reset();
  env.run();
  EXPECT_FALSE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name holding) (eq ?p:param-values (create$ b1)))"));
  EXPECT_TRUE(has_fact("((?p domain-fact))", "(eq ?p:name handempty)"));
  EXPECT_TRUE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b1)))"));
  env.assert_fact("(apply-action 1)");
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name holding) (eq ?p:param-values (create$ b1)))"));
  EXPECT_FALSE(has_fact("((?p domain-fact))", "(eq ?p:name handempty)"));
  EXPECT_FALSE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b1)))"));
}

TEST_F(BlocksworldDomainTest, ApplyContradictingEffectsWithDifferentParams)
{
  env.reset();
  env.run();
  EXPECT_TRUE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b1)))"));
  EXPECT_FALSE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b2)))"));
  env.assert_fact("(apply-action 2)");
  env.run();
  EXPECT_FALSE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b1)))"));
  EXPECT_TRUE(has_fact("((?p domain-fact))",
        "(and (eq ?p:name clear) (eq ?p:param-values (create$ b2)))"));
}

TEST_F(DomainTest, PreconditionWithConstant)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
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

TEST_F(DomainTest, PreconditionWithConstantInSecondSlot)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
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

TEST_F(DomainTest, PreconditionWithUnknownParameter)
{
  env.reset();
  env.assert_fact("(plan-action"
                  " (id 1)"
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
  EXPECT_TRUE(has_fact("((?e domain-error))"));
}
