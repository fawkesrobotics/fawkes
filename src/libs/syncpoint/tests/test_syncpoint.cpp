/***************************************************************************
 *  test_syncpoint.cpp - SyncPoint Unit Test
 *
 *  Created: Wed Jan 22 11:17:43 2014
 *  Copyright  2014-2015  Till Hofmann
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

#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include <string>

#include <libs/syncpoint/syncpoint.h>
#include <libs/syncpoint/syncpoint_manager.h>
#include <libs/syncpoint/exceptions.h>

#include <core/utils/refptr.h>

using namespace fawkes;
using namespace std;

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
      string id1 = "/id1";
      string id2 = "/id2";
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
    SyncPointManagerTest()
    {
      manager = new SyncPointManager();

      pthread_attr_init(&attrs);

    }

    /**
     * Deinitialize the test class
     */
    virtual ~SyncPointManagerTest()
    {
      pthread_attr_destroy(&attrs);
    }

    /**
     * A Pointer to a SyncPointManager
     */
    RefPtr<SyncPointManager> manager;

    /** Thread attributes */
    pthread_attr_t attrs;
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

TEST_F(SyncPointTest, SyncPointSets)
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
  ASSERT_EQ(manager->get_syncpoints().size(), 0u);
  manager->get_syncpoint("test", "/test/1");
  ASSERT_EQ(manager->get_syncpoints().size(), 1u);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))), 1u);
  manager->get_syncpoint("test", "/test/2");
  ASSERT_EQ(manager->get_syncpoints().size(), 2u);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))), 1u);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/2"))), 1u);
  manager->get_syncpoint("test2", "/test/1");
  ASSERT_EQ(manager->get_syncpoints().size(), 2u);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))), 1u);
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/2"))), 1u);
}

TEST_F(SyncPointManagerTest, WatcherSet)
{
  ASSERT_NO_THROW(manager->get_syncpoint("component 1", "/test"));
  ASSERT_NO_THROW(manager->get_syncpoint("component 2", "/test"));
  ASSERT_NO_THROW(manager->get_syncpoint("component 3", "/test"));
  ASSERT_THROW(manager->get_syncpoint("component 1", "/test"), SyncPointAlreadyOpenedException);

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
  pthread_create(&thread1, &attrs, call_wait, (void *)sp);
  // make sure the other thread is first
  usleep(10000);
  ASSERT_THROW(sp_ref->wait("component"), SyncPointMultipleWaitCallsException);
  pthread_cancel(thread1);
  pthread_join(thread1, NULL);
}


/** struct used for multithreading tests */
struct waiter_thread_params {
    /** SyncPointManager passed to the thread */
    RefPtr<SyncPointManager> manager;
    /** Thread number */
    uint thread_nr;
    /** Number of wait calls the thread should make */
    uint num_wait_calls;
    /** Name of the SyncPoint */
    string sp_identifier;
};


/** get a SyncPoint and wait for it */
void * start_waiter_thread(void * data) {
  waiter_thread_params *params = (waiter_thread_params *)data;
  char *comp;
  asprintf(&comp, "component %u", params->thread_nr);
  string component = comp;
  free(comp);
  RefPtr<SyncPoint> sp = params->manager->get_syncpoint(component, params->sp_identifier);
  for (uint i = 0; i < params->num_wait_calls; i++) {
    sp->wait(component);
  }
  pthread_exit(NULL);
}
/** Create multiple threads which will all call get_syncpoint
 *  for the same SyncPoint. Do not wait for the SyncPoint but return
 *  immediately.
 */
TEST_F(SyncPointManagerTest, MultipleManagerRequests)
{
  uint num_threads = 50;
  pthread_t threads[num_threads];
  waiter_thread_params *params[num_threads];
  string sp_identifier = "/test/sp1";
  for (uint i = 0; i < num_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    params[i]->num_wait_calls = 0;
    params[i]->sp_identifier = sp_identifier;
    pthread_create(&threads[i], &attrs, start_waiter_thread, params[i]);
    pthread_yield();
  }

  for (uint i = 0; i < num_threads; i++) {
    pthread_join(threads[i], NULL);
    delete params[i];
  }
}


/** start multiple threads and let them wait.
 *  This just tests whether there are any segfaults.
 *  No assertions are made.
 */
TEST_F(SyncPointManagerTest, ParallelWaitCalls)
{
  uint num_threads = 50;
  uint num_wait_calls = 10;
  pthread_t threads[num_threads];
  waiter_thread_params *params[num_threads];
  string sp_identifier = "/test/sp1";
  for (uint i = 0; i < num_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    params[i]->num_wait_calls = num_wait_calls;
    params[i]->sp_identifier = sp_identifier;
    pthread_create(&threads[i], &attrs, start_waiter_thread, params[i]);
    pthread_yield();
  }

  usleep(10000);
  for (uint i = 0; i < num_threads; i++) {
    pthread_cancel(threads[i]);
    ASSERT_EQ(0,pthread_join(threads[i], NULL));
    delete params[i];
  }
}

/** start multiple threads, let them wait for a SyncPoint,
 * emit the SyncPoint and verify that they all returned
 */
TEST_F(SyncPointManagerTest, ParallelWaitsReturn)
{

  uint num_threads = 50;
  uint num_wait_calls = 10;
  pthread_t threads[num_threads];
  waiter_thread_params *params[num_threads];
  string sp_identifier = "/test/sp1";
  for (uint i = 0; i < num_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    params[i]->num_wait_calls = num_wait_calls;
    params[i]->sp_identifier = sp_identifier;
    pthread_create(&threads[i], &attrs, start_waiter_thread, params[i]);
    usleep(10000);
  }

  string component = "emitter";
  RefPtr<SyncPoint> sp = manager->get_syncpoint(component, sp_identifier);
  for (uint i = 0; i < num_wait_calls; i++) {
    sp->emit(component);
    usleep(10000);
  }

  sleep(1);
  for (uint i = 0; i < num_threads; i++) {
    ASSERT_EQ(0, pthread_tryjoin_np(threads[i], NULL));
    delete params[i];
  }
}

/** start multiple threads, let them wait for a SyncPoint,
 * but don't emit the SyncPoint. Verify that they have not returned
 */
TEST_F(SyncPointManagerTest, WaitDoesNotReturnImmediately)
{
  uint num_threads = 50;
  pthread_t threads[num_threads];
  waiter_thread_params *params[num_threads];
  for (uint i = 0; i < num_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    params[i]->num_wait_calls = 1;
    params[i]->sp_identifier = "/test/sp1";
    pthread_create(&threads[i], &attrs, start_waiter_thread, params[i]);
  }

  sleep(1);
  for (uint i = 0; i < num_threads; i++) {
    EXPECT_EQ(EBUSY, pthread_tryjoin_np(threads[i], NULL));
    pthread_cancel(threads[i]);
    ASSERT_EQ(0, pthread_join(threads[i], NULL));
    delete params[i];
  }
}
