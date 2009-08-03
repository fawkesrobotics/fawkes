
/***************************************************************************
 *  read_write_lock.cpp - Read Write Lock
 *
 *  Generated: Thu Sep 15 00:10:54 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/read_write_lock.h>

#include <pthread.h>
#include <cstring>

namespace fawkes {


/// @cond INTERNALS
class ReadWriteLockData
{
 public:
  pthread_rwlock_t rwlock;
};
/// @endcond


/** @class ReadWriteLock core/threading/read_write_lock.h
 * Read/write lock to allow multiple readers but only a single writer
 * on the resource at a time.
 * This can be used if you have a value that only a few writers modify but
 * several readers use. In this case the readers can read all at the same
 * time as long as there is no writer modifying the value.
 *
 * @ingroup Threading
 * @ingroup FCL
 * @see example_rwlock.cpp
 *
 * @author Tim Niemueller
 */


/** Constructor
 * @param policy The read/write lock policy to use. The default is to
 * prefer writers.
 */
ReadWriteLock::ReadWriteLock(ReadWriteLockPolicy policy)
{
  rwlock_data = new ReadWriteLockData();

#if defined __USE_UNIX98 || defined __USE_XOPEN2K
  pthread_rwlockattr_t attr;
  pthread_rwlockattr_init( &attr );

  switch (policy) {
  case RWLockPolicyPreferWriter:
    pthread_rwlockattr_setkind_np( &attr, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP );
    break;
  case RWLockPolicyPreferReader:
    pthread_rwlockattr_setkind_np( &attr, PTHREAD_RWLOCK_PREFER_READER_NP );
    break;
  }

  pthread_rwlock_init( &(rwlock_data->rwlock), &attr );
#else
  pthread_rwlock_init( &(rwlock_data->rwlock), NULL );
#endif
}


/** Destructor */
ReadWriteLock::~ReadWriteLock()
{
  pthread_rwlock_destroy( &(rwlock_data->rwlock) );
  delete rwlock_data;
}


/** Aquire a reader lock.
 * This will aquire the lock for reading. Multiple readers can aquire the
 * lock at the same time. But never when a writer has the lock.
 * This method will block until the lock has been aquired.
 */
void
ReadWriteLock::lock_for_read()
{
  pthread_rwlock_rdlock( &(rwlock_data->rwlock) );
}


/** Aquire a writer lock.
 * This will aquire the lock for writing. Only a single writer at a time
 * will be allowed to aquire the lock.
 * This method will block until the lock has been aquired.
 */
void
ReadWriteLock::lock_for_write()
{
  pthread_rwlock_wrlock( &(rwlock_data->rwlock) );
}


/** Tries to aquire a reader lock.
 * This will try to aquire the lock for reading. This will succeed if
 * no writer has aquired the lock already. Multiple readers may aquire the
 * lock.
 * @return true, if the lock could be aquired, false otherwise.
 */
bool
ReadWriteLock::try_lock_for_read()
{
  return ( pthread_rwlock_tryrdlock( &(rwlock_data->rwlock) ) == 0 );
}


/** Tries to aquire a writer lock.
 * This will try to aquire the lock for writing. This will succeed if the
 * read/write lock is currently unlocked. No other threads may hold this lock
 * at the same time. Neither for writing nor for reading.
 * @return true, if the lock has been aquired, false otherwise.
 */
bool
ReadWriteLock::try_lock_for_write()
{
  return ( pthread_rwlock_trywrlock( &(rwlock_data->rwlock) ) == 0 );
}


/** Release the lock.
 * Releases the lock, no matter whether it was locked for reading or writing.
 */
void
ReadWriteLock::unlock()
{
  pthread_rwlock_unlock( &(rwlock_data->rwlock) );
}


} // end namespace fawkes
