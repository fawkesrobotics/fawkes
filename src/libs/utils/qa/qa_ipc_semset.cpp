
/***************************************************************************
 *  qa_ipc_semset.h - QA for IPC semaphore sets
 *
 *  Generated: Tue Sep 19 17:00:23 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

// Do not include in api reference
///@cond QA

#include <utils/ipc/semset.h>

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>

#define FATHER_LOCK 0
#define CHILD_LOCK  1

using namespace std;
using namespace fawkes;

bool quit;

void
signal_handler(int signum)
{
  cout << "Signal handler called" << endl;
  quit = true;
}

int
main( int argc, char **argv )
{
  quit = false;
  signal(SIGINT, signal_handler);

  pid_t child_pid;

  if ((child_pid = fork()) == 0) {
    // child

    SemaphoreSet *s2 = new SemaphoreSet(".", 'A', 2, false, false);

    while ( !s2->valid() ) {
      // wait for father to open up semaphore, could also set create to true
      // in constructor call
      usleep(100000);
    }

    while ( ! quit ) {

      cout << "Child: Unlocking child lock" << endl;
      s2->unlock(CHILD_LOCK);

      cout << "Child: Waiting for father lock to become ready" << endl;
      s2->lock(FATHER_LOCK);
      cout << "Child: Father lock aquired, unlocking" << endl;
      s2->unlock(FATHER_LOCK);

      cout << "Child: Sleeping" << endl;
      usleep(521342);
      cout << "Child: Locking child lock" << endl;
      s2->lock(CHILD_LOCK);
      cout << "Child: Sleeping again" << endl;
      usleep(12323);
    }
    
    cout << "Child: Destroying s2" << endl;
    delete s2;

  } else {
    // father

    // Will be used by father
    // Semaphore set with two semaphores, but zero at the beginning
    SemaphoreSet *s1 = new SemaphoreSet(".", 'A', 2, true, true);

    while ( ! quit ) {
      cout << "Father: Unlocking father lock" << endl;
      s1->unlock(FATHER_LOCK);

      cout << "Father: Waiting for child lock to become ready" << endl;
      s1->lock(CHILD_LOCK);
      cout << "Father: Child lock aquired, unlocking" << endl;
      s1->unlock(CHILD_LOCK);

      cout << "Father: Sleeping" << endl;
      usleep(821342);
      cout << "Father: Locking father lock" << endl;
      s1->lock(FATHER_LOCK);
      cout << "Father: again" << endl;
      usleep(52323);
    }

    cout << "Father: Waiting for child to exit" << endl;
    int status;
    waitpid(child_pid, &status, 0);

    cout << "Father: Destroying s1" << endl;
    delete s1;
  }

  return 0;
}


/// @endcond
