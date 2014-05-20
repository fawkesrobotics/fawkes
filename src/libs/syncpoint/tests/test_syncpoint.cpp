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

#include <unistd.h>

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

// helper function used for testing wait()
void * call_wait(void *data)
{
  SyncPoint * sp = (SyncPoint *)(data);
  sp->wait("component");
  return NULL;
}

TEST_F(SyncPointManagerTest, MultipleWaits)
{
  RefPtr<SyncPoint> sp_ref = manager->get_syncpoint("component", "/test/sp1");
  SyncPoint * sp = *sp_ref;
  pthread_t thread1;
  pthread_create(&thread1, NULL, call_wait, (void *)sp);
  // make sure the other thread is first
  usleep(100);
  ASSERT_THROW(sp_ref->wait("component"), SyncPointMultipleWaitCallsException);
  pthread_cancel(thread1);
}

/** struct used for multithreading tests */
struct waiter_thread_params {
    /** SyncPointManager passed to the thread */
    RefPtr<SyncPointManager> manager;
    /** Thread number */
    uint thread_nr;
};

/** get a SyncPoint and wait for it */
void * start_waiter_thread(void * data) {
  waiter_thread_params *params = (waiter_thread_params *)data;
  char component[40];
  sprintf(component, "component %u", params->thread_nr);
  RefPtr<SyncPoint> sp = params->manager->get_syncpoint(component, "/test/sp1");
  sp->wait(component);
  return NULL;
}

TEST_F(SyncPointManagerTest, ParallelWaitCalls)
{
  uint num_threads = 100;
  pthread_t threads[num_threads];
  waiter_thread_params *params[num_threads];
  for (uint i = 0; i < num_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    pthread_create(&threads[i], NULL, start_waiter_thread, params[i]);
  }

  usleep(100);
  for (uint i = 0; i < num_threads; i++) {
    pthread_cancel(threads[i]);
    delete params[i];
  }
}
