
/***************************************************************************
 *  example_mutx_count.cpp - Example for counting with multiple threads and
 *                           protecting the count variable with a mutex
 *
 *  Generated: Thu Sep 14 16:29:37 2006
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

#include <core/threading/thread.h>
#include <core/threading/mutex.h>

#include <iostream>

//  By default do not include examples in API documentation
/// @cond EXAMPLES

using namespace std;
using namespace fawkes;

#define WASTETIME  \
  for ( unsigned int i = 0; i < 1000000; i++) { \
    unsigned int j;				\
    j = i + i;					\
  }


/** Simple test class for counting with multiple threads.
 * Compile the test program and let it run. You will see that even after only a short time
 * the values for the protected and the unprotected count variables differ.
 */
class ExampleMutexCountThread : public Thread
{
public:
  /** Constructor
   * @param s Short identifier, printed first in output
   * @param m The mutex used to protect count variable
   * @param mutex_count Protected count variable
   * @param non_mutex_count Unprotected count variable
   * @param sleep_time Variable sleep time at end of thread
   */
  ExampleMutexCountThread(string s,
			  Mutex *m, unsigned int *mutex_count, unsigned int *non_mutex_count,
			  unsigned int sleep_time)
    : Thread("ExampMutexCountThread", Thread::OPMODE_CONTINUOUS)
  {
    this->s   = s;
    this->sl  = sl;
    this->slt = sleep_time;
    this->m   = m;
    this->mc  = mutex_count;
    this->nmc = non_mutex_count;
  }

  /** Where the action happens
   */
  virtual void loop()
  {
    // unprotected modification, another thread could modify the value while
    // we waste time
    unsigned int n = *nmc;
    n++;
    sleep(0);
    WASTETIME;
    *nmc = n;
      
    // protected modification, no other thread can modify the value as long as
    // we have the lock
    if ( m != NULL )  m->lock();
    unsigned o = *mc;
    o++;
    sleep(0);
    WASTETIME;
    *mc = o;
    if ( m != NULL )  m->unlock();
    
    // Out is not mutexed, can lead to wrong printouts, try it (happens rarely)!
    cout << s << ": mutex: " << *mc << "(non-mutex: " << *nmc << ")" << endl;

    if ( sl )   usleep(slt);

    test_cancel();
  }

 private:
  string s;
  bool   sl;
  unsigned int slt;
  Mutex *m;
  unsigned int *mc;
  unsigned int *nmc;
};


int
main(int argc, char **argv)
{

  Mutex *m = new Mutex();

  unsigned int mutex_count = 0;
  unsigned int non_mutex_count = 0;

  ExampleMutexCountThread *t1 = new ExampleMutexCountThread("t1", m, &mutex_count, &non_mutex_count, 1000);
  ExampleMutexCountThread *t2 = new ExampleMutexCountThread("t2", m, &mutex_count, &non_mutex_count, 10000);
  ExampleMutexCountThread *t3 = new ExampleMutexCountThread("t3", m, &mutex_count, &non_mutex_count, 100000);

  t1->start();
  t2->start();
  t3->start();

  // Wait for all threads to finish
  t1->join();
  t2->join();
  t3->join();

  delete t1;
  delete t2;
  delete t3;
  delete m;
}


/// @endcond
