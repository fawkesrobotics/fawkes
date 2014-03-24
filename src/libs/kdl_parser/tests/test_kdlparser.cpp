/***************************************************************************
 *  kdlparser-test.cpp - SyncPoint Unit Test
 *
 *  Created on Mon Mar 24 15:47:30 2014
 *  Copyright (C) 2014
 *
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

#include <gtest/gtest.h>
#include <libs/kdl_parser/kdl_parser.h>
#include <kdl/tree.hpp>

#include <string>

using namespace fawkes;
using namespace fawkes::kdl_parser;
using namespace std;

/** Test Class for KDLParser */
class KDLParserTest : public ::testing::Test
{

  protected:
    /** Load the robot description and initialize the tree */
    virtual void SetUp()
    {
      urdf_file = SRCDIR"/caesar.urdf";
      ASSERT_TRUE(tree_from_file(urdf_file, tree));
    }
    /** The result of parsing the URDF file */
    KDL::Tree tree;
    /** The URDF file path */
    string urdf_file;
};



TEST_F(KDLParserTest, FileDoesNotExist) {
  string filename = "invalidFileName";
  KDL::Tree tree;
  ASSERT_ANY_THROW(tree_from_file(filename, tree));
}

TEST_F(KDLParserTest, NumberOfJoints) {
  EXPECT_EQ((unsigned int)9, tree.getNrOfJoints());
  EXPECT_EQ((unsigned int)37, tree.getNrOfSegments());
}

TEST_F(KDLParserTest, ChainToKatanaFinger) {
  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain("/base_link", "katana_l_finger_link", chain));
  EXPECT_EQ(9, chain.getNrOfSegments());
  EXPECT_EQ(6, chain.getNrOfJoints());
}

TEST_F(KDLParserTest, RootSegment) {
  EXPECT_EQ("/base_link", tree.getRootSegment()->first);
  EXPECT_EQ(tree.getSegment("/base_link"), tree.getRootSegment());
}
