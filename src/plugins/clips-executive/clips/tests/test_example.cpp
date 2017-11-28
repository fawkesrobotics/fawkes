/***************************************************************************
 *  test_example.cpp - An exemplary CLIPS unit test
 *
 *  Created: Mon 25 Sep 2017 16:47:31 CEST 16:47
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

/** Simple Test class that shows how to do unit testing for CLIPS. */
class SimpleCLIPSTest : public CLIPSTest
{
  protected:
    /** Set up the test environment. */
    virtual void SetUp() {
      LoadCLIPSFiles({ "test_example.clp" });
    }
};

TEST_F(SimpleCLIPSTest, SimpleTest) {
  env.assert_fact("(testfact)");
  env.assert_fact("(foo bar 4.2)");
  CLIPS::Fact::pointer fact_p = env.get_facts();
  env.run();
  EXPECT_TRUE(has_fact("((?t testtempl))", "(eq ?t:name foo)"));
  EXPECT_TRUE(has_fact("((?f foo))", "(eq ?f:implied (create$ bar 4.2))"));
  EXPECT_FALSE(has_fact("((?t testtempl))", "(eq ?t:name bar)"));
  EXPECT_FALSE(has_ordered_fact("foo"));
  EXPECT_TRUE(has_ordered_fact("foo", { "bar", 4.2 }));
}
