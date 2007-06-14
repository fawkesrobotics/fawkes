
/***************************************************************************
 *  qa_time.cpp - QA app for watch, clock, timesoure, ...
 *
 *  Generated: Sun June 03 22:33:22 2007
 *  Copyright  2007  Daniel Beck 
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <utils/system/clock.h>
#include <utils/system/watch.h>

#include <unistd.h>

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  Watch watch;
  Time time;

  time = watch.clock_time();
  cout << "Current clock time is " << time << endl;

  watch.start(&time);
  cout << "Starting watch at " << time << endl;
  sleep(1);

  cout << "Watch is running for " << watch.watch_time() << endl;
  sleep(1);

  cout << "Watch is running for " << watch.watch_time() << endl;
  sleep(1);

  watch.pause(&time);
  cout << "Start pausing at "<< time << endl;
  sleep(1);

  cout << "Watch is running for " << watch.watch_time() << endl;
  sleep(1);

  cout << "Watch is running for " << watch.watch_time() << endl;
  sleep(1);

  watch.start(&time);
  cout << "Resuming at "<< time << endl;
  sleep(1);

  watch.stop(&time);
  cout << "Stopping watch at "<< time << endl;

  cout << "Watch is running for " << watch.watch_time() << endl;
  
  time = watch.clock_time();
  cout << "Current clock time is " << time << endl;

  Clock::finalize();

  return 0;
}
