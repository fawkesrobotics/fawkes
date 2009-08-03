
/***************************************************************************
 *  qa_timebug.cpp - QA app to find a potential bug related to the Time class
 *
 *  Created: Tue Dec 18 10:38:30 2007
 *  Copyright  2007  Tim Niemueller 
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

/// @cond QA

#include <core/threading/thread.h>
#include <utils/time/clock.h>
#include <utils/time/time.h>
#include <utils/time/wait.h>
#include <utils/system/signal.h>

#include <unistd.h>

#include <iostream>

using namespace std;
using namespace fawkes;


class QaTestWait
{
public:
  QaTestWait()
  {
    __clock = Clock::instance();
    __until = new Time();
  }


  void mark_start()
  {
    __clock->get_time(__until);
    *__until += (long int)30000;
  }

  void wait()
  {
    Time now;
    printf("Now at %p\n", &now);
    __clock->get_time(&now);
    usleep(0);
    long int remaining_usec = (*__until - now).in_usec();
    while ( remaining_usec > 0 ) {
      usleep(remaining_usec);
      __clock->get_time(&now);
      remaining_usec = (*__until - now).in_usec();
      //remaining_usec = 0;
    }
  }

  Clock *__clock;
  Time  *__until;
};

class QaSignalHandler : public SignalHandler
{
public:
  QaSignalHandler(Thread *thread)
  {
    this->thread = thread;
  }

  virtual void handle_signal(int signum)
  {
    thread->cancel();
  }

  Thread *thread;
};

class QaTestThread : public Thread
{
public:
  QaTestThread() : Thread("QaTestThread")
  {
    timewait = new TimeWait(Clock::instance(), 30000);
    testwait = new QaTestWait();
  }

  virtual void loop()
  {
    printf("Loop running\n");
    timewait->mark_start();
    timewait->wait();
    //testwait->mark_start();
    //testwait->wait();
  }

  QaTestWait *testwait;
  TimeWait *timewait;
};

int main(int argc, char** argv)
{
  QaTestThread t;
  t.start();

  QaSignalHandler h(&t);
  SignalManager::register_handler(SIGINT, &h);

  t.join();

  return 0;
}

/// @endcond
