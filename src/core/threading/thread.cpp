
/***************************************************************************
 *  thread.cpp - implementation of threads, based on pthreads
 *
 *  Generated: Thu Sep 14 13:26:39 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/thread.h>

#include <pthread.h>


bool
Thread::start()
{
  int err;
  if ( (err = pthread_create(&thread_id, NULL, Thread::entry, this)) != 0) {
    // An error occured
    return false;
  }

  return true;
}


/* static */  void *
Thread::entry(void *pthis)
{
  Thread *t = (Thread *)pthis;
  t->run();
  return NULL;
}


void
Thread::exit()
{
  pthread_exit(NULL);
}


void
Thread::join()
{
  void *dont_care;
  pthread_join(thread_id, &dont_care);
}


void
Thread::detach()
{
  pthread_detach(thread_id);
}


void
Thread::cancel()
{
  pthread_cancel(thread_id);
}


void
Thread::test_cancel()
{
  pthread_testcancel();
}


bool
Thread::operator==(const Thread &thread)
{
  return ( pthread_equal(thread_id, thread.thread_id) != 0 );
}


void
Thread::run()
{
  forever {
    loop();
    test_cancel();
  }
}


void
Thread::loop()
{
}
