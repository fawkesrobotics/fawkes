/***************************************************************************
 *  syncpoint-test.cpp - SyncPoint Unit Test
 *
 *  Created: Wed Jan 22 11:17:43 2014
 *  Copyright  2014  Till Hofmann
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

#include <libs/syncpoint/syncpoint.h>
#include <libs/syncpoint/exceptions.h>
#include <libs/syncpoint/syncpoint_manager.h>

#include <core/utils/refptr.h>

#include <pthread.h>
#include <baseapp/run.h>

#include <set>

using namespace fawkes;

/** @class SyncPointTest
 * Test class for SyncPoint
 * This class tests basic functionality of SyncPoints
 */
class SyncPointTest : public ::testing::Test
{

  protected:
    /**
     * Initialize the test class
     */
    virtual void SetUp()
    {
      const char * id1 = "/id1";
      const char * id2 = "/id2";
      //const char * id3 = "/id3";
      sp1 = new SyncPoint(id1);
      sp2 = new SyncPoint(id1);
      sp3 = new SyncPoint(id2);
    }


    /**@{*/
    /**
     * Syncpoints for testing purposes
     */
    RefPtr<SyncPoint> sp1;
    RefPtr<SyncPoint> sp2;
    RefPtr<SyncPoint> sp3;
    /**@}*/
};

/** @class SyncPointManagerTest
 * Test class for SyncPointManager
 * This class tests basic functionality of the SyncPointManager
 */
class SyncPointManagerTest : public ::testing::Test
{
  protected:
    /**
     * Initialize the test class
     */
    virtual void SetUp()
    {
      manager = new SyncPointManager();
    }

    /**
     * A Pointer to a SyncPointManager
     */
    RefPtr<SyncPointManager> manager;
};

TEST_F(SyncPointTest, CreateSyncPoint)
{
  ASSERT_TRUE(*sp1 != NULL);
}

TEST_F(SyncPointTest, Equals)
{
  // RefPtr<SyncPoint>
  ASSERT_NE(sp1, sp2);
  // SyncPoint*
  ASSERT_NE(*sp1, *sp2);
  // SyncPoint
  ASSERT_EQ(**sp1, **sp2);
}

TEST_F(SyncPointTest, LessThan)
{
  ASSERT_LT(**sp1, **sp3);
  ASSERT_FALSE(**sp3 < **sp1);
  ASSERT_FALSE(**sp1 < **sp2);
  ASSERT_FALSE(**sp2 < **sp1);
}

TEST_F(SyncPointTest, Sets)
{
  using namespace std;
  set<RefPtr<SyncPoint>, SyncPointSetLessThan > sp_set;
  pair<set<RefPtr<SyncPoint> >::iterator, bool> ret;

  // insert sp1
  ret = sp_set.insert(sp1);
  ASSERT_TRUE(ret.second);
  ASSERT_EQ(sp1->get_identifier(), (*(ret.first))->get_identifier());

  // insert sp3
  ret = sp_set.insert(sp3);
  ASSERT_TRUE(ret.second);
  ASSERT_EQ(sp3->get_identifier(), (*(ret.first))->get_identifier());

  // insert sp1 again
  ret = sp_set.insert(sp1);
  ASSERT_FALSE(ret.second);
  ASSERT_EQ(sp1->get_identifier(), (*(ret.first))->get_identifier());

  // insert sp2 (same as sp1)
  ret = sp_set.insert(sp2);
  ASSERT_FALSE(ret.second);
  ASSERT_EQ(sp2->get_identifier(), (*(ret.first))->get_identifier());
}

TEST_F(SyncPointManagerTest, SyncPointManager)
{
  ASSERT_EQ(manager->get_syncpoints().size(), 0);
  manager->get_syncpoint("test", "/test/1");
  ASSERT_EQ(manager->get_syncpoints().size(), 1);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))), 1);
  manager->get_syncpoint("test", "/test/2");
  ASSERT_EQ(manager->get_syncpoints().size(), 2);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))), 1);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/2"))), 1);
  manager->get_syncpoint("test2", "/test/1");
  ASSERT_EQ(manager->get_syncpoints().size(), 2);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))), 1);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/2"))), 1);
}

TEST_F(SyncPointTest, EmptyIdentifier)
{
  ASSERT_THROW(sp1 = new SyncPoint(""), SyncPointInvalidIdentifierException);
}

TEST_F(SyncPointTest, InvalidIdentifier)
{
  ASSERT_THROW(sp1 = new SyncPoint("invalid"), SyncPointInvalidIdentifierException);

}

TEST_F(SyncPointManagerTest, SyncPointManagerExceptions) {
  RefPtr<SyncPoint> invalid_sp;
  ASSERT_THROW(invalid_sp = manager->get_syncpoint("", "/test/sp1"),
      SyncPointInvalidComponentException);

  // make sure syncpoint_manager doesn't catch the exceptions thrown by SyncPoint
  ASSERT_THROW(invalid_sp = manager->get_syncpoint("waiter", ""),
      SyncPointInvalidIdentifierException);
  ASSERT_THROW(invalid_sp = manager->get_syncpoint("waiter", "invalid"),
        SyncPointInvalidIdentifierException);


}
