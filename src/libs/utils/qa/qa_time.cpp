
/***************************************************************************
 *  qa_time.cpp - QA app for watch, clock, timesoure, ...
 *
 *  Generated: Sun June 03 22:33:22 2007
 *  Copyright  2007  Daniel Beck 
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

#include <utils/time/clock.h>
#include <utils/time/watch.h>

#include <unistd.h>

#include <iostream>

using namespace std;
using namespace fawkes;

int main(int argc, char** argv)
{
  Clock *clock = Clock::instance();

  Watch watch(clock);
  Time time(clock);

  time = watch.clock_time();
  cout << "Current clock time is " << time.str() << endl;

  watch.start(&time);
  cout << "Starting watch at " << time.str() << endl;
  sleep(1);

  cout << "Watch is running for " << watch.watch_time().str() << endl;
  sleep(1);

  cout << "Watch is running for " << watch.watch_time().str() << endl;
  sleep(1);

  watch.pause(&time);
  cout << "Start pausing at "<< time.str() << endl;
  sleep(1);

  cout << "Watch is running for " << watch.watch_time().str() << endl;
  sleep(1);

  cout << "Watch is running for " << watch.watch_time().str() << endl;
  sleep(1);

  watch.start(&time);
  cout << "Resuming at "<< time.str() << endl;
  sleep(1);

  watch.stop(&time);
  cout << "Stopping watch at "<< time.str() << endl;

  cout << "Watch is running for " << watch.watch_time().str() << endl;
  
  time = watch.clock_time();
  cout << "Current clock time is " << time.str() << endl;

  Clock::finalize();

  return 0;
}
