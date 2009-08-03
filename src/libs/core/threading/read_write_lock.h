
/***************************************************************************
 *  read_write_lock.h - Read Write Lock
 *
 *  Generated: Thu Sep 15 00:07:41 2006
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

#ifndef __CORE_THREADING_READ_WRITE_LOCK_H_
#define __CORE_THREADING_READ_WRITE_LOCK_H_

namespace fawkes {


class ReadWriteLockData;

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

  ReadWriteLock(ReadWriteLockPolicy policy = RWLockPolicyPreferWriter);

  virtual ~ReadWriteLock();

  void lock_for_read();
  void lock_for_write();
  bool try_lock_for_read();
  bool try_lock_for_write();
  void unlock();

 private:
  ReadWriteLockData *rwlock_data;
};


} // end namespace fawkes

#endif
