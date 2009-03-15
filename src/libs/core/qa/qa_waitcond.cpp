
/***************************************************************************
 *  example_waitcond.cpp - wait condition example program
 *
 *  Created: Sat Mar 01 15:13:44 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: example_barrier.cpp 210 2007-06-13 14:01:49Z tim $
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

// Do not mention in API doc
/// @cond EXAMPLES

#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <core/exception.h>
#include <core/threading/mutex.h>

#include <iostream>
#include <string>

using namespace std;
using namespace fawkes;

typedef enum {
  WAITER,
  WAKER,
  BUGGER
} threadmode_t;

class ExampleWaitCondThread : public Thread
{
 public:
  ExampleWaitCondThread(threadmode_t mode, string tname,
			WaitCondition *waitcond, unsigned int sleep_time)
    : Thread(tname.c_str(), Thread::OPMODE_CONTINUOUS)
  {
    this->mode       = mode;
    this->waitcond   = waitcond;
    this->sleep_time = sleep_time;
  }

  virtual void loop()
  {
    if ( mode == WAITER ) {
      usleep( sleep_time );
      cout << name() << ": Waiting for waker" << endl;
      try {
	waitcond->wait();
	cout << name() << ": Woken up" << endl;
      } catch (Exception &e) {
	cout << name() << ": EXCEPTION" << endl;
	e.print_trace();
      }
    } else if (mode == BUGGER) {
      usleep( sleep_time );
      Mutex mutex;
      try {
	waitcond->wait(&mutex);
      } catch (Exception &e) {
	cout << name() << ": failed as expected" << endl;
	e.print_trace();
      }
      mode = WAITER;
    } else {
      usleep( sleep_time );
      cout << name() << ": Waking waiter" << endl;
      waitcond->wake_all();
      cout << name() << ": Woke waiter" << endl;
    }
  }

 private:
  threadmode_t mode;
  WaitCondition *waitcond;
  unsigned int sleep_time;

};


int
main(int argc, char **argv)
{
  WaitCondition *w = new WaitCondition();

  ExampleWaitCondThread *t1 = new ExampleWaitCondThread(WAITER, "waiter1", w, 1000000);
  ExampleWaitCondThread *t2 = new ExampleWaitCondThread(WAITER, "waiter2", w, 1200000);
  ExampleWaitCondThread *tw = new ExampleWaitCondThread(WAKER, "waker", w, 6458642);
  ExampleWaitCondThread *tb = new ExampleWaitCondThread(BUGGER, "bugger", w, 3458642);

  t1->start();
  t2->start();
  tw->start();
  tb->start();

  t1->join();
  t2->join();
  tw->join();
  tb->join();

  delete t1;
  delete t2;
  delete tw;
  delete tb;

  delete w;
}


/// @endcond
