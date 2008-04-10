
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

#include <iostream>
#include <string>

using namespace std;

class ExampleWaitCondThread : public Thread
{
 public:
  ExampleWaitCondThread(string pp,
		       WaitCondition *waitcond, unsigned int sleep_time)
    : Thread("ExampleWaitCondThread", Thread::OPMODE_CONTINUOUS)
  {
    this->pp         = pp;
    this->waitcond   = waitcond;
    this->sleep_time = sleep_time;
  }

  virtual void loop()
  {
    if ( pp == "waiter" ) {
      cout << pp << ": Waiting for waker" << endl;
      waitcond->wait();
    } else {
      usleep( sleep_time );
      cout << pp << ": Waking waiter" << endl;
      waitcond->wake_all();
      cout << pp << ": Woke waiter" << endl;
    }
  }

 private:
  WaitCondition *waitcond;
  unsigned int sleep_time;
  string pp;

};


int
main(int argc, char **argv)
{
  WaitCondition *w = new WaitCondition();

  ExampleWaitCondThread *t1 = new ExampleWaitCondThread("waiter", w, 0);
  ExampleWaitCondThread *t2 = new ExampleWaitCondThread("waker", w, 6458642);

  t1->start();
  t2->start();

  t1->join();
  t2->join();

  delete t1;
  delete t2;

  delete w;
}


/// @endcond
