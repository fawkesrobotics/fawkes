
/***************************************************************************
 *  example_waitcond_serialize.cpp - example application for using condition
 *                                   variables to serialize threads
 *
 *  Generated: Thu Sep 14 21:43:30 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

/// @cond EXAMPLES

#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <core/threading/mutex.h>

#include <iostream>

using namespace std;
using namespace fawkes;

/** Small example hread serializing with other threads using a wait condition.
 * Run the program and see them printing out numbers serialized.
 */
class ExampleWaitCondThread : public Thread
{
 public:
  /** Constructor
   * @param wc Wait condition
   * @param m Mutex that is locked for the condition variable
   * @param val Pointer to the current value
   * @param actval Activation value when this thread becomes active
   * @param maxval Maximum value when to reset the value
   */
  ExampleWaitCondThread(WaitCondition *wc, Mutex *m, int *val, int actval, int maxval)
    : Thread("ExampleWaitCondThread", Thread::OPMODE_CONTINUOUS)
  {
    this->wc     = wc;
    this->m      = m;
    this->val    = val;
    this->actval = actval;
    this->maxval = maxval;
  }

  /** Action!
   */
  virtual void loop()
  {
    m->lock();
    while (*val != actval) {
      wc->wait();
    }
    cout << *val << " called" << endl;
    *val += 1;
    if ( *val > maxval ) {
      *val = 0;
    }
    // unlock mutex inside wait condition
    m->unlock();
    
    // Cannot call wake_one() here since result is unpredictable and if not
    // the next thread is woken up we will end up in a deadlock. So every
    // thread has to check if it's his turn -> use wake_all()
    wc->wake_all();
  }

 private:
  WaitCondition *wc;
  Mutex         *m;

  int           *val;
  int            actval;
  int            maxval;

};

/* This small app uses a condition variable to serialize
 * a couple of threads
 */
int
main(int argc, char **argv)
{

  int val = 0;

  Mutex *m = new Mutex();
  WaitCondition *wc = new WaitCondition(m);

  ExampleWaitCondThread *t1 = new ExampleWaitCondThread(wc, m, &val, 0, 4);
  ExampleWaitCondThread *t2 = new ExampleWaitCondThread(wc, m, &val, 1, 4);
  ExampleWaitCondThread *t3 = new ExampleWaitCondThread(wc, m, &val, 2, 4);
  ExampleWaitCondThread *t4 = new ExampleWaitCondThread(wc, m, &val, 3, 4);
  ExampleWaitCondThread *t5 = new ExampleWaitCondThread(wc, m, &val, 4, 4);

  t1->start();
  t2->start();
  t3->start();
  t4->start();
  t5->start();

  t1->join();
  t2->join();
  t3->join();
  t4->join();
  t5->join();

  delete t5;
  delete t4;
  delete t3;
  delete t2;
  delete t1;
  delete wc;
  delete m;

  return 0;
}


/// @endcond
