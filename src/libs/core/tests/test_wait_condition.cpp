/***************************************************************************
 *  test_wait_condition.cpp - WaitCondition Unit Test
 *
 *  Created: Sat Jan 24 15:12:42 2015
 *  Copyright  2015  Till Hofmann
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
#include <unistd.h>
#include <time.h>

#include <core/threading/wait_condition.h>

using namespace fawkes;

/** The parameters passed to the threads. */
struct thread_params
{
  /** The thread's wait condition. */
  WaitCondition * cond;
};

/** @class WaitConditionTest
 *  Test class for WaitConditions
 *  This class tets basic functionality of WaitConditions,
 *  mainly thread cancellation problems
 */
class WaitConditionTest : public ::testing::Test
{
protected:
  WaitConditionTest()
  : cond(new WaitCondition()),
    num_threads(2)
  {
  }
  virtual ~WaitConditionTest() {
    delete cond;
  }

  /** Start threads with the given function, cancel the threads
   *  and assert they have terminated.
   *  @param thread_func The function the threads are started with.
   */
  void start_test(void * (*thread_func) (void *)) {
    pthread_t threads[num_threads];
    thread_params *params[num_threads];
    for (uint i = 0; i < num_threads; i++) {
      params[i] = new thread_params();
      params[i]->cond = cond;
      pthread_create(&threads[i], NULL, thread_func, params[i]);
      pthread_yield();
    }

    usleep(1000);
    for (uint i = 0; i < num_threads; i++) {
      pthread_cancel(threads[i]);
      struct timespec ts;
      ASSERT_NE(-1, clock_gettime(CLOCK_REALTIME, &ts));
      // give the thread two seconds to terminate
      ts.tv_sec += 2;
      ASSERT_EQ(0, pthread_timedjoin_np(threads[i], NULL, &ts));
      delete params[i];
    }
  }

private:
  WaitCondition *cond;
  const uint num_threads;
};


void * start_waiter_thread(void * args)
{
  thread_params *params = (thread_params *) args;
  params->cond->wait();
  pthread_exit(NULL);
}

void * start_abstimed_waiter_thread(void * args)
{
  thread_params *params = (thread_params *) args;
  struct timespec ts;
  EXPECT_NE(-1, clock_gettime(CLOCK_REALTIME, &ts));
  ts.tv_sec += 5;
  params->cond->abstimed_wait(ts.tv_sec, 0);
  pthread_exit(NULL);
}

void * start_reltimed_waiter_thread(void * args)
{
  thread_params *params = (thread_params *) args;
  params->cond->reltimed_wait(10, 0);
  pthread_exit(NULL);
}

TEST_F(WaitConditionTest, CancelWaitingThread)
{
  start_test(start_waiter_thread);
}

TEST_F(WaitConditionTest, CancelAbsTimedWaitingThread)
{
  start_test(start_abstimed_waiter_thread);
}

TEST_F(WaitConditionTest, CancelRelTimedWaitingThread)
{
  start_test(start_reltimed_waiter_thread);
}

