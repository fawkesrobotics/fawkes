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

class DomainTest : public CLIPSTest
{
  protected:
    virtual void SetUp() {
      LoadCLIPSFiles(clips_files);
    }
    vector<string> clips_files = { "../domain.clp" };
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
  EXPECT_TRUE(has_ordered_fact("is-satisfied", { "neg-on-table" }));
  EXPECT_TRUE(has_ordered_fact("is-satisfied", { "pick-up-precond" }));
}

TEST_F(BlocksworldDomainTest, NegativePreconditionIsNotSatisfied)
{
  env.reset();
  env.assert_fact("(predicate (name ontable) (parameters b1))");
  env.run();
  EXPECT_FALSE(has_ordered_fact("is-satisfied", { "neg-on-table" }));
  EXPECT_FALSE(has_ordered_fact("is-satisfied", { "pick-up-precond" }));
}

TEST_F(BlocksworldDomainTest, GroundingWithMultipleParameters)
{
  env.reset();
  env.run();
  EXPECT_FALSE(has_fact("((?p atomic-precondition))",
                        "(eq ?p:grounded partially)"));
  EXPECT_TRUE(has_fact("((?p atomic-precondition))",
                       "(and (eq ?p:predicate on) "
                            "(eq ?p:grounded yes) "
                            "(eq ?p:parameters (create$ b1 b2)))"));
  EXPECT_TRUE(has_ordered_fact("is-satisfied", { "unstack-precond" }));
}

TEST_F(DomainTest, Typing)
{
  env.reset();
  env.assert_fact("(dom-object (name thing))");
  env.assert_fact("(obj-type (name moveable-obj))");
  env.assert_fact("(obj-type (name cup) (super-type moveable-obj))");
  env.assert_fact("(dom-object (name c1) (obj-type cup))");
  env.run();
  EXPECT_TRUE(has_ordered_fact("obj-is-of-type", { "thing", "object" }));
  EXPECT_TRUE(has_ordered_fact("obj-is-of-type", { "c1", "cup" }));
  EXPECT_TRUE(has_ordered_fact("obj-is-of-type", { "c1", "moveable-obj" }));
  EXPECT_TRUE(has_ordered_fact("obj-is-of-type", { "c1", "object" }));
}
