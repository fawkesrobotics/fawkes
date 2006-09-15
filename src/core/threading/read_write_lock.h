
/***************************************************************************
 *  read_write_lock.h - Read Write Lock
 *
 *  Generated: Thu Sep 15 00:07:41 2006
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

#ifndef __CORE_THREADING_READ_WRITE_LOCK_H_
#define __CORE_THREADING_READ_WRITE_LOCK_H_

class ReadWriteLockData;

/** Read/write lock to allow multiple readers but only a single writer
 * on the resource at a time.
 * This can be used if you have a value that only a few writers modify but
 * several readers use. In this case the readers can read all at the same
 * time as long as there is no writer modifying the value.
 *
 * @author Tim Niemueller
 */
class ReadWriteLock
{
 public:

  /** The policy to use for the read/write lock.
   */
  enum ReadWriteLockPolicy {
    RWLockPolicyPreferWriter,    /**< Prefer writers over readers. A writer
				  * is granted prefered access to the lock.
				  * This means that the writer can aquire
				  * the lock as soon as all readers unlocked
				  * it not matter if there are readers queued
				  * and waiting for the lock. This is the
				  * default behaviour. Not with multiple
				  * writers you can run into the problem of
				  * reader starvation.
				  */
    RWLockPolicyPreferReader     /**< Prefer readers over writers. This is
				  * similar to the writer preference. Readers
				  * will be allowed to aquire the lock no
				  * matter if there is a writer enqueued for
				  * the lock. Not that with many readers
				  * (depending on the time they aquire the
				  * lock this can already start with two
				  * or three readers) you can run into the
				  * problem of writer starvation: the writer
				  * can never aquire the lock.
				  */
  };

  /** Constructor
   * @param policy The read/write lock policy to use. The default is to
   * prefer writers.
   */
  ReadWriteLock(ReadWriteLockPolicy policy = RWLockPolicyPreferWriter);

  /** Destructor
   */
  ~ReadWriteLock();

  /** Aquire a reader lock.
   * This will aquire the lock for reading. Multiple readers can aquire the
   * lock at the same time. But never when a writer has the lock.
   * This method will block until the lock has been aquired.
   */
  void lockForRead();

  /** Aquire a writer lock.
   * This will aquire the lock for writing. Only a single writer at a time
   * will be allowed to aquire the lock.
   * This method will block until the lock has been aquired.
   */
  void lockForWrite();

  /** Tries to aquire a reader lock.
   * This will try to aquire the lock for reading. This will succeed if
   * no writer has aquired the lock already. Multiple readers may aquire the
   * lock.
   * @return true, if the lock could be aquired, false otherwise.
   */
  bool tryLockForRead();

  /** Tries to aquire a writer lock.
   * This will try to aquire the lock for writing. This will succeed if the
   * read/write lock is currently unlocked. No other threads may hold this lock
   * at the same time. Neither for writing nor for reading.
   * @return true, if the lock has been aquired, false otherwise.
   */
  bool tryLockForWrite();

  /** Release the lock.
   * Releases the lock, no matter whether it was locked for reading or writing.
   */
  void unlock();

 private:
  ReadWriteLockData *rwlock_data;
};



#endif
