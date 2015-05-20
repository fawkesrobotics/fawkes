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

/** @class SyncBarrierTest
 *  Test SyncBarriers
 */
class SyncBarrierTest : public SyncPointManagerTest
{
protected:
  /** Constructor. */
  SyncBarrierTest()
  {
  }
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
  ASSERT_EQ(0u, manager->get_syncpoints().size());
  manager->get_syncpoint("test", "/test/1");
  ASSERT_EQ(3u, manager->get_syncpoints().size());
  ASSERT_EQ(manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))), 1u);
  manager->get_syncpoint("test2", "/test/2");
  ASSERT_EQ(4u, manager->get_syncpoints().size());
  ASSERT_EQ(1u, manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))));
  ASSERT_EQ(1u, manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/2"))));
  manager->get_syncpoint("test3", "/test/1");
  ASSERT_EQ(4u, manager->get_syncpoints().size());
  ASSERT_EQ(1u, manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/1"))));
  ASSERT_EQ(1u, manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test/2"))));
  ASSERT_EQ(1u, manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/"))));
  ASSERT_EQ(1u, manager->get_syncpoints().count(RefPtr<SyncPoint>(new SyncPoint("/test"))));
}

TEST_F(SyncPointManagerTest, WatcherSet)
{
  ASSERT_NO_THROW(manager->get_syncpoint("component 1", "/test"));
  ASSERT_NO_THROW(manager->get_syncpoint("component 2", "/test"));
  ASSERT_NO_THROW(manager->get_syncpoint("component 3", "/test"));
}

/** Test what happens if we acquire a SyncPoint, release it, and then acquire it
 * again. If release_syncpoint works properly, this should not throw. Otherwise,
 * we would expect a SyncPointAlreadyOpenedException
 */
TEST_F(SyncPointManagerTest, ReleaseAndReacquire)
{
  string comp = "component";
  string id = "/test/sp1";
  RefPtr<SyncPoint> sp = manager->get_syncpoint(comp, id);
  set<RefPtr<SyncPoint>, SyncPointSetLessThan > syncpoints = manager->get_syncpoints();
  ASSERT_EQ(1, syncpoints.count(RefPtr<SyncPoint>(new SyncPoint("/test"))));
  for (set<RefPtr<SyncPoint> >::const_iterator sp_it = syncpoints.begin();
      sp_it != syncpoints.end(); sp_it++) {
    EXPECT_EQ(1, (*sp_it)->get_watchers().count(comp))
        << "for component '" << comp << "' and SyncPoint '"
        << (*sp_it)->get_identifier() << "'";
  }
  manager->release_syncpoint(comp, sp);
  for (set<RefPtr<SyncPoint> >::const_iterator sp_it = syncpoints.begin();
      sp_it != syncpoints.end(); sp_it++) {
    EXPECT_EQ(0, (*sp_it)->get_watchers().count(comp)) << "for component '"
        << comp << "' and SyncPoint '" << (*sp_it)->get_identifier() << "'";
  }
  ASSERT_NO_THROW(manager->get_syncpoint(comp, id));
}

TEST_F(SyncPointTest, EmptyIdentifier)
{
  ASSERT_THROW(sp1 = new SyncPoint(""), SyncPointInvalidIdentifierException);
}

TEST_F(SyncPointTest, InvalidIdentifier)
{
  EXPECT_THROW(sp1 = new SyncPoint("invalid"), SyncPointInvalidIdentifierException);
  EXPECT_NO_THROW(sp1 = new SyncPoint("/"));
  EXPECT_THROW(sp1 = new SyncPoint("/test/"), SyncPointInvalidIdentifierException);
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

TEST_F(SyncPointManagerTest, SyncPointHierarchyRegisteredWatchers)
{
  string comp = "component1";
  string id = "/test/sp1";
  RefPtr<SyncPoint> sp = manager->get_syncpoint(comp, "/test/sp1");
  set<RefPtr<SyncPoint>, SyncPointSetLessThan > syncpoints = manager->get_syncpoints();
  set<RefPtr<SyncPoint>>::iterator sp_test_it = syncpoints.find(RefPtr<SyncPoint>(new SyncPoint("/test")));
  set<RefPtr<SyncPoint>>::iterator sp_root_it = syncpoints.find(RefPtr<SyncPoint>(new SyncPoint("/")));
  ASSERT_NE(syncpoints.end(), sp_test_it);
  ASSERT_NE(syncpoints.end(), sp_root_it);
  RefPtr<SyncPoint> sp_test = *sp_test_it;
  RefPtr<SyncPoint> sp_root = *sp_root_it;
  EXPECT_EQ(1, syncpoints.count(sp_test));
  EXPECT_EQ(1, syncpoints.count(sp_root));
  EXPECT_EQ(1, sp->get_watchers().count(comp));
  EXPECT_EQ(1, sp_test->get_watchers().count(comp));
  EXPECT_EQ(0, sp_test->get_watchers().count(id));
  EXPECT_EQ(1, sp_root->get_watchers().count(comp));
  EXPECT_EQ(0, sp_root->get_watchers().count(id));
  EXPECT_EQ(0,
      sp_root->get_watchers().count(
          sp_test->get_identifier()));

  manager->release_syncpoint(comp, sp);
  EXPECT_EQ(0, sp_test->get_watchers().count(id));
}

TEST_F(SyncPointManagerTest, SyncPointComponentRegistersForMultipleSyncPoints)
{
  string comp = "component1";
  string sp1_id = "/test/sp1";
  string sp2_id = "/test/sp2";
  RefPtr<SyncPoint> sp1 = manager->get_syncpoint(comp, sp1_id);
  // the following should not throw
  // if it does, registering for the predecessor '/test' may be broken
  RefPtr<SyncPoint> sp2 = manager->get_syncpoint(comp, sp2_id);
  RefPtr<SyncPoint> predecessor = *manager->get_syncpoints().find(
      RefPtr<SyncPoint>(new SyncPoint("/test")));
  EXPECT_EQ(1, sp1->get_watchers().count(comp))
      << comp << " is not registered for " << sp1->get_identifier()
      << ", but should be!";
  EXPECT_EQ(1, sp2->get_watchers().count(comp))
      << comp << " is not registered for " << sp2->get_identifier()
      << ", but should be!";
  EXPECT_EQ(1, predecessor->get_watchers().count(comp))
      << comp << " is not registered for " << predecessor->get_identifier()
      << ", but should be!";

  manager->release_syncpoint(comp, sp1);
  EXPECT_EQ(1, sp2->get_watchers().count(comp));
  EXPECT_EQ(1, predecessor->get_watchers().count(comp))
      << comp << " is not registered for " << predecessor->get_identifier()
      << ", but should be!";
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
    uint thread_nr = 0;
    /** Number of wait calls the thread should make */
    uint num_wait_calls;
    /** Name of the SyncPoint */
    string sp_identifier;
    /** Name of the component */
    string component = "";
};


/** get a SyncPoint and wait for it */
void * start_waiter_thread(void * data) {
  waiter_thread_params *params = (waiter_thread_params *)data;
  string component = params->component;
  if (component == "") {
    char *comp;
    asprintf(&comp, "component %u", params->thread_nr);
    component = comp;
    free(comp);
  }
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
    ASSERT_LE(manager->get_syncpoints().size(), 3u);
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
    ASSERT_LE(manager->get_syncpoints().size(), 3u);
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
  sp->register_emitter(component);
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

/**
 * Test the SyncPoint hierarchy.
 * This creates a SyncPoint, an emitter and waiters which wait for the
 * SyncPoint's predecessor, the predecessor's predecessor (grandparent),
 * and the root SyncPoint ("/").
 */
TEST_F(SyncPointManagerTest, SyncPointHierarchy)
{
  vector<string> identifiers = { "/test/topic", "/test", "/", "/other/topic" };
  uint num_threads = identifiers.size();
  pthread_t threads[num_threads];
  waiter_thread_params *params[num_threads];
  for (uint i = 0; i < num_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    params[i]->num_wait_calls = 1;
    params[i]->sp_identifier = identifiers.at(i);
    pthread_create(&threads[i], &attrs, start_waiter_thread, params[i]);
  }

  usleep(10000);
  RefPtr<SyncPoint> sp = manager->get_syncpoint("emitter", "/test/topic/sp");
  sp->register_emitter("emitter");
  sp->emit("emitter");
  usleep(10000);

  /* The first waiters should be unblocked */
  for (uint i = 0; i < num_threads - 1 ; i++) {
    ASSERT_EQ(0, pthread_tryjoin_np(threads[i], NULL));
    delete params[i];
  }

  /* The last waiter should still wait */
  pthread_t last_thread = threads[num_threads-1];
  EXPECT_EQ(EBUSY, pthread_tryjoin_np(last_thread, NULL));
  pthread_cancel(last_thread);
  ASSERT_EQ(0, pthread_join(last_thread, NULL));
}

/** Emit a barrier without registering */
TEST_F(SyncBarrierTest, EmitWithoutRegister)
{
  string component = "emitter";
  RefPtr<SyncPoint> barrier = manager->get_syncpoint(component, "/test/barrier");
  ASSERT_THROW(barrier->emit(component), SyncPointNonEmitterCalledEmitException);
}

/** Register multiple times
 * This is allowed, but the component should then also emit multiple times */
TEST_F(SyncBarrierTest, MultipleRegisterCalls)
{
  string component = "emitter";
  RefPtr<SyncPoint> barrier = manager->get_syncpoint(component, "/test/barrier");
  EXPECT_NO_THROW(barrier->register_emitter(component));
  EXPECT_NO_THROW(barrier->register_emitter(component));
}

/** get a SyncBarrier and wait for it */
void * start_barrier_waiter_thread(void * data) {
  waiter_thread_params *params = (waiter_thread_params *)data;
  char *comp;
  asprintf(&comp, "component %u", params->thread_nr);
  string component = comp;
  free(comp);
  RefPtr<SyncPoint> sp;
  sp = params->manager->get_syncpoint(component, params->sp_identifier);
  for (uint i = 0; i < params->num_wait_calls; i++) {
    sp->wait(component, SyncPoint::WAIT_FOR_ALL);
  }
  pthread_exit(NULL);
}

/** get a SyncBarrier, register as emitter and emit */
void * start_barrier_emitter_thread(void * data) {
  waiter_thread_params *params = (waiter_thread_params *)data;
  char *comp;
  asprintf(&comp, "emitter %u", params->thread_nr);
  string component = comp;
  free(comp);
  RefPtr<SyncPoint> sp;
  EXPECT_NO_THROW(sp = params->manager->get_syncpoint(component, params->sp_identifier));
  sp->register_emitter(component);
  for (uint i = 0; i < params->num_wait_calls; i++) {
    sp->emit(component);
  }
  pthread_exit(NULL);
}

/** Helper class which registers and emits a given SyncBarrier */
class Emitter {
public:
  /** Constructor.
   *  @param identifier The identifier of this emitter.
   *  @param syncbarrier The identifier of the SyncBarrier to register for.
   *  @param manager Pointer to the SyncPointManager to use.
   */
  Emitter(string identifier, string syncbarrier, RefPtr<SyncPointManager> manager)
  : identifier_(identifier),
    manager_(manager)
  {
    barrier_ = manager->get_syncpoint(identifier_, syncbarrier);
    barrier_->register_emitter(identifier_);
  }

  /** Destructor. */
  virtual ~Emitter()
  {
    barrier_->unregister_emitter(identifier_);
    manager_->release_syncpoint(identifier_, barrier_);
  }

  /** emit the SyncBarrier */
  void emit()
  {
    barrier_->emit(identifier_);
  }

private:
  string identifier_;
  RefPtr<SyncPoint> barrier_;
  RefPtr<SyncPointManager> manager_;
};


/** Barrier: wait() returns immediately if no emitter is registered */
TEST_F(SyncBarrierTest,WaitWithNoRegisteredEmitter)
{
  string barrier_id = "/test/barrier";
  RefPtr<SyncPoint> barrier = manager->get_syncpoint("main loop", barrier_id);
  const uint num_waiter_threads = 1;
  const uint num_wait_calls = 1;
  pthread_t waiter_threads[num_waiter_threads];
  waiter_thread_params *params[num_waiter_threads];
  for (uint i = 0; i < num_waiter_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    params[i]->num_wait_calls = num_wait_calls;
    params[i]->sp_identifier = barrier_id;
    pthread_create(&waiter_threads[i], &attrs, start_barrier_waiter_thread, params[i]);
    usleep(10000);
  }
  for (uint i = 0; i < num_waiter_threads; i++) {
    ASSERT_EQ(0, pthread_tryjoin_np(waiter_threads[i], NULL));
    delete params[i];
  }
}

/** Start multiple threads, let them wait for a SyncBarrier,
 *  also have two threads registered as emitter.
 *  Let the first thread emit the barrier, assert the waiters did not unblock,
 *  then let the second thread emit.
 *  This tests the fundamental difference to a SyncPoint: With a SyncPoint,
 *  wait() returns if the SyncPoint is emitted by one component.
 *  With a SyncBarrier, all registered emitters need to emit the SyncBarrier
 *  before wait() returns.
 */
TEST_F(SyncBarrierTest, WaitForAllEmitters)
{

  string barrier_id = "/test/barrier";
  Emitter em1("emitter 1", barrier_id, manager);
  Emitter em2("emitter 2", barrier_id, manager);

  RefPtr<SyncPoint> barrier = manager->get_syncpoint("main loop", barrier_id);

  const uint num_waiter_threads = 50;
  const uint num_wait_calls = 1;
  pthread_t waiter_threads[num_waiter_threads];
  waiter_thread_params *params[num_waiter_threads];
  for (uint i = 0; i < num_waiter_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    params[i]->num_wait_calls = num_wait_calls;
    params[i]->sp_identifier = barrier_id;
    pthread_create(&waiter_threads[i], &attrs, start_barrier_waiter_thread, params[i]);
    usleep(10000);
  }

  sleep(1);
  for (uint i = 0; i < num_waiter_threads; i++) {
    EXPECT_EQ(EBUSY, pthread_tryjoin_np(waiter_threads[i], NULL));
  }

  em1.emit();

  sleep(1);
  for (uint i = 0; i < num_waiter_threads; i++) {
    EXPECT_EQ(EBUSY, pthread_tryjoin_np(waiter_threads[i], NULL));
  }

  em1.emit();

  sleep(1);
  for (uint i = 0; i < num_waiter_threads; i++) {
    EXPECT_EQ(EBUSY, pthread_tryjoin_np(waiter_threads[i], NULL));
  }

  em2.emit();

  sleep(1);
  for (uint i = 0; i < num_waiter_threads; i++) {
    ASSERT_EQ(0, pthread_tryjoin_np(waiter_threads[i], NULL));
    delete params[i];
  }
}


/** two barriers, emit the first one. Only the threads waiting on the first
 *  barrier should unblock
 */
TEST_F(SyncBarrierTest, BarriersAreIndependent)
{
  string barrier1_id = "/test/barrier1";
  string barrier2_id = "/test/barrier2";
  Emitter em1("em1", barrier1_id, manager);
  Emitter em2("em2", barrier2_id, manager);

  RefPtr<SyncPoint> barrier1 = manager->get_syncpoint("m1",
    barrier1_id);

  RefPtr<SyncPoint> barrier2 = manager->get_syncpoint("m2",
    barrier2_id);

  const uint num_waiter_threads = 50;
  const uint num_wait_calls = 1;
  pthread_t waiter_threads1[num_waiter_threads];
  waiter_thread_params *params1[num_waiter_threads];
  for (uint i = 0; i < num_waiter_threads; i++) {
    params1[i] = new waiter_thread_params();
    params1[i]->manager = manager;
    params1[i]->thread_nr = i;
    params1[i]->num_wait_calls = num_wait_calls;
    params1[i]->sp_identifier = barrier1_id;
    pthread_create(&waiter_threads1[i], &attrs, start_barrier_waiter_thread,
      params1[i]);
    usleep(10000);
  }

  pthread_t waiter_threads2[num_waiter_threads];
  waiter_thread_params *params2[num_waiter_threads];
  for (uint i = 0; i < num_waiter_threads; i++) {
    params2[i] = new waiter_thread_params();
    params2[i]->manager = manager;
    params2[i]->thread_nr = num_waiter_threads + i;
    params2[i]->num_wait_calls = num_wait_calls;
    params2[i]->sp_identifier = barrier2_id;
    pthread_create(&waiter_threads2[i], &attrs, start_barrier_waiter_thread,
      params2[i]);
    usleep(10000);
  }

  sleep(1);
  for (uint i = 0; i < num_waiter_threads; i++) {
    EXPECT_EQ(EBUSY, pthread_tryjoin_np(waiter_threads1[i], NULL));
  }

  for (uint i = 0; i < num_waiter_threads; i++) {
    EXPECT_EQ(EBUSY, pthread_tryjoin_np(waiter_threads2[i], NULL));
  }

  em1.emit();

  sleep(1);
  for (uint i = 0; i < num_waiter_threads; i++) {
    ASSERT_EQ(0, pthread_tryjoin_np(waiter_threads1[i], NULL));
    delete params1[i];
  }

  for (uint i = 0; i < num_waiter_threads; i++) {
    EXPECT_EQ(EBUSY, pthread_tryjoin_np(waiter_threads2[i], NULL));
  }

  em2.emit();

  sleep(1);
  for (uint i = 0; i < num_waiter_threads; i++) {
    ASSERT_EQ(0, pthread_tryjoin_np(waiter_threads2[i], NULL));
    delete params2[i];
  }
}

/**
 * Test the SyncBarrier hierarchy, similar to the SyncPoint hierarchy test.
 * This creates a SyncBarrier, an emitter and waiters which wait for the
 * SyncBarrier's predecessor, the predecessor's predecessor (grandparent),
 * and the root SyncBarrier ("/").
 */
TEST_F(SyncBarrierTest, SyncBarrierHierarchy)
{
  Emitter em1("emitter 1", "/test/topic/b1", manager);
  Emitter em2("emitter 2", "/test/topic/b2", manager);
  Emitter em3("emitter 3", "/other/topic", manager);

  vector<string> identifiers = { "/test/topic", "/test", "/", "/other/topic" };
  uint num_threads = identifiers.size();
  pthread_t threads[num_threads];
  waiter_thread_params *params[num_threads];
  for (uint i = 0; i < num_threads; i++) {
    params[i] = new waiter_thread_params();
    params[i]->manager = manager;
    params[i]->thread_nr = i;
    params[i]->num_wait_calls = 1;
    params[i]->sp_identifier = identifiers.at(i);
    pthread_create(&threads[i], &attrs, start_barrier_waiter_thread, params[i]);
  }

  usleep(10000);

  for (uint i = 0; i < num_threads; i++) {
    ASSERT_EQ(EBUSY, pthread_tryjoin_np(threads[i], NULL));
  }
  em1.emit();
  usleep(10000);
  for (uint i = 0; i < num_threads; i++) {
    ASSERT_EQ(EBUSY, pthread_tryjoin_np(threads[i], NULL));
  }
  em2.emit();
  usleep(10000);
  /* The first waiters should be unblocked */
  for (uint i = 0; i < num_threads - 2 ; i++) {
    ASSERT_EQ(0, pthread_tryjoin_np(threads[i], NULL));
    delete params[i];
  }
  /* The last two waiters should still be waiting */
  for (uint i = num_threads - 2; i < num_threads; i++) {
    EXPECT_EQ(EBUSY, pthread_tryjoin_np(threads[i], NULL));
    pthread_cancel(threads[i]);
    ASSERT_EQ(0, pthread_join(threads[i], NULL));
    delete params[i];
  }
}

/** One component registers as emitter for two syncpoints, two other components
 *  wait for the first and second syncpoint respectively.
 *  Then, the first component unregisters for the first syncpoint.
 *  Test whether it is still registered for the second syncpoint.
 *  A third waiter waits for the predecessor syncpoint and should also still be
 *  waiting after the emitter has unregistered for the first syncpoint.
 */
TEST_F(SyncPointManagerTest, OneEmitterRegistersForMultipleSyncPointsHierarchyTest)
{
  string id_sp1 = "/test/sp1";
  string id_sp2 = "/test/sp2";
  string id_sp_pred = "/test";
  string id_emitter = "component_emitter";
  string id_waiter1 = "component_waiter1";
  string id_waiter2 = "component_waiter2";
  string id_waiter3 = "component_waiter_on_predecessor";

  RefPtr<SyncPoint> sp1 = manager->get_syncpoint(id_emitter, id_sp1);
  RefPtr<SyncPoint> sp2 = manager->get_syncpoint(id_emitter, id_sp2);
  manager->get_syncpoint(id_waiter1, id_sp1);
  manager->get_syncpoint(id_waiter2, id_sp2);
  RefPtr<SyncPoint> pred = manager->get_syncpoint(id_waiter3, id_sp_pred);
  sp1->register_emitter(id_emitter);
  sp2->register_emitter(id_emitter);
  EXPECT_EQ(1, sp1->get_emitters().count(id_emitter));
  EXPECT_EQ(1, sp2->get_emitters().count(id_emitter));
  // this should be 2 as the emitter has registered twice
  EXPECT_EQ(2, pred->get_emitters().count(id_emitter));


  waiter_thread_params *params1 = new waiter_thread_params();
  params1->manager = manager;
  params1->component = id_waiter1;
  params1->num_wait_calls = 1;
  params1->sp_identifier = id_sp1;

  waiter_thread_params *params2 = new waiter_thread_params();
  params2->manager = manager;
  params2->component = id_waiter2;
  params2->num_wait_calls = 1;
  params2->sp_identifier = id_sp2;

  waiter_thread_params *params3 = new waiter_thread_params();
  params3->manager = manager;
  params3->component = id_waiter3;
  params3->num_wait_calls = 1;
  params3->sp_identifier = id_sp_pred;

  pthread_t pthread1;
  pthread_create(&pthread1, &attrs, start_barrier_waiter_thread, params1);
  pthread_t pthread2;
  pthread_create(&pthread2, &attrs, start_barrier_waiter_thread, params2);
  pthread_t pthread3;
  pthread_create(&pthread3, &attrs, start_barrier_waiter_thread, params3);

  usleep(10000);
  sp1->emit(id_emitter);
  usleep(10000);
  EXPECT_EQ(0, pthread_tryjoin_np(pthread1, NULL));
  EXPECT_EQ(EBUSY, pthread_tryjoin_np(pthread2, NULL));
  // this should be EBUSY as the component has registered twice for '/test'
  // and thus should emit '/test' also twice (by hierarchical emit calls)
  EXPECT_EQ(EBUSY, pthread_tryjoin_np(pthread3, NULL));
  sp2->emit(id_emitter);
  usleep(10000);
  EXPECT_EQ(0, pthread_tryjoin_np(pthread2, NULL));
  EXPECT_EQ(0, pthread_tryjoin_np(pthread3, NULL));

  sp2->unregister_emitter(id_emitter);
  EXPECT_EQ(1, sp1->get_emitters().count(id_emitter));
  EXPECT_EQ(0, sp2->get_emitters().count(id_emitter));
  EXPECT_EQ(1, pred->get_emitters().count(id_emitter));

  pthread_create(&pthread1, &attrs, start_barrier_waiter_thread, params1);
  pthread_create(&pthread2, &attrs, start_barrier_waiter_thread, params2);
  pthread_create(&pthread3, &attrs, start_barrier_waiter_thread, params3);

  usleep(10000);
  EXPECT_EQ(EBUSY, pthread_tryjoin_np(pthread1, NULL));
  ASSERT_EQ(0, pthread_tryjoin_np(pthread2, NULL));
  EXPECT_EQ(EBUSY, pthread_tryjoin_np(pthread3, NULL));

  sp1->emit(id_emitter);
  usleep(10000);
  ASSERT_EQ(0, pthread_tryjoin_np(pthread1, NULL));
  ASSERT_EQ(0, pthread_tryjoin_np(pthread3, NULL));
  delete params1;
  delete params2;
  delete params3;

}
