
/***************************************************************************
 *  scoped_rwlock.h - Scoped read/write lock
 *
 *  Created: Mon Jan 10 11:42:56 2011
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_THREADING_SCOPED_RWLOCK_H_
#define __CORE_THREADING_SCOPED_RWLOCK_H_

#include <core/utils/refptr.h>

namespace fawkes {

class ReadWriteLock;

class ScopedRWLock
{
 public:
  /** What to lock for. */
  typedef enum {
    LOCK_WRITE,	///< Lock for writing
    LOCK_READ	///< Lock for reading
  } LockType;

  ScopedRWLock(RefPtr<ReadWriteLock> rwlock, LockType lock_type = LOCK_WRITE,
               bool initially_lock = true);
  ScopedRWLock(ReadWriteLock *rwlock, LockType lock_type = LOCK_WRITE,
               bool initially_lock = true);
  ~ScopedRWLock();

  void relock();
  void unlock();

 private:
  LockType               __lock_type;
  bool                   __locked;
  RefPtr<ReadWriteLock>  __refrwlock;
  ReadWriteLock         *__rawrwlock;
};


} // end namespace fawkes

#endif
