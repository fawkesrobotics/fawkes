
/***************************************************************************
 *  example_rwlock.cpp - Example for an ReadWriteeLock
 *
 *  Generated: Fri Sep 15 11:53:23 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

// Do not mention in API doc
/// @cond EXAMPLES

#include <core/threading/thread.h>
#include <core/threading/read_write_lock.h>

#include <iostream>
#include <string>

using namespace std;

/** Example writer thread, will aquire lock and increment
 * the value. Will print out a notice if it has
 * to wait for readers.
 */

class ExampleRWLockWriterThread : public Thread
{
 public:
  ExampleRWLockWriterThread(ReadWriteLock *rwlock, int *val, unsigned int sleep_time)
  {
    this->rwlock     = rwlock;
    this->val        = val;
    this->sleep_time = sleep_time;
  }

  /** Action!
   */
  virtual void run()
  {
    forever {
      if ( ! rwlock->tryLockForWrite() ) {
	cout << "Writer: Readers on lock, waiting for release" << endl;
	rwlock->lockForWrite();
	// aquired the lock
      }
      cout << "Writer: aquired lock" << endl;
      (*val)++;
      usleep(sleep_time);
      rwlock->unlock();

      // Give other threads a chance to get in
      usleep(0);
    }
  }

 private:
  ReadWriteLock *rwlock;
  int           *val;
  unsigned int   sleep_time;
};


/** Example reader thread, will aquire reader lock and print out
 * the current value. Will print a notice if it has to wait for a writer
 * to unlock the lock.
 */
class ExampleRWLockReaderThread : public Thread
{
 public:
  ExampleRWLockReaderThread(string pp,
			    ReadWriteLock *rwlock, int *val, unsigned int sleep_time)
  {
    this->pp         = pp;
    this->rwlock     = rwlock;
    this->val        = val;
    this->sleep_time = sleep_time;
  }

  virtual void loop()
  {
    if ( ! rwlock->tryLockForRead() ) {
      cout << "Reader (" << pp << "): Writer on lock, waiting for release" << endl;
      rwlock->lockForRead();
    }
    cout << "Reader (" << pp << "): aquired lock" << endl;
    cout << "Reader (" << pp << "): val=" << *val << endl;
    usleep(sleep_time);
    cout << "Reader (" << pp << "): Unlocking" << endl;
    rwlock->unlock();

    // Give other threads a chance to get in
    usleep(0);
  }

 private:
  string         pp;
  ReadWriteLock *rwlock;
  int           *val;
  unsigned int   sleep_time;
};


int
main(int argc, char **argv)
{
  int val = 0;

  ReadWriteLock *rwlock = new ReadWriteLock();

  ExampleRWLockWriterThread *tw  = new ExampleRWLockWriterThread(rwlock, &val, 100000);
  ExampleRWLockReaderThread *tr1 = new ExampleRWLockReaderThread("r1", rwlock, &val, 234234);
  ExampleRWLockReaderThread *tr2 = new ExampleRWLockReaderThread("r2", rwlock, &val, 156743);
  ExampleRWLockReaderThread *tr3 = new ExampleRWLockReaderThread("r3", rwlock, &val, 623442);
  ExampleRWLockReaderThread *tr4 = new ExampleRWLockReaderThread("r4", rwlock, &val, 455345);

  tw->start();
  tr1->start();
  tr2->start();
  tr3->start();
  tr4->start();

  tw->join();
  tr1->join();
  tr2->join();
  tr3->join();
  tr4->join();

  delete tw;
  delete tr1;
  delete tr2;
  delete tr3;
  delete tr4;
  delete rwlock;

  return 0;
}


/// @endcond
