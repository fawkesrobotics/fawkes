
/***************************************************************************
 *  mutex.h - Mutex
 *
 *  Generated: Thu Sep 14 16:58:49 2006
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

#ifndef __CORE_THREADING_MUTEX_H_
#define __CORE_THREADING_MUTEX_H_

// Forward declaration to avoid header inclusion
class MutexData;

class WaitCondition;

/** Mutex mutual exclusion lock.
 * This class is used in a multi-threading environment to lock access to
 * resources. This is needed to prevent two threads from modifying a value
 * at the same time or to prevent a thread from getting a dirty copy of
 * a piece of data (the reader reads while a writer is writing, this could
 * leave the data in a state where the reader reads half of the new and half
 * of the old data).
 *
 * As a rule of thumb you should lock the mutex as short as possible and as
 * long as needed. Locking the mutex too long will lead in a bad performance
 * of the multi-threaded application because many threads are waiting for
 * the lock and are not doing anything useful.
 * If you do not lock enough code (and so serialize it) it will cause pain
 * and errors.
 *
 * @see example_mutex_count.cpp
 *
 * @author Tim Niemueller
 */
class Mutex
{
  friend class WaitCondition;

 public:
  /** Constructor
   */
  Mutex();

  /** Destructor
   */
  ~Mutex();

  /** Lock this mutex.
   */
  void lock();

  /** Tries to lock the mutex.
   * This can also be used to check if a mutex is locked. The code for this
   * can be:
   *
   * @code
   * bool locked = false;
   * if ( mutex->tryLock() ) {
   *   mutex->unlock();
   *   locked = true;
   * }
   * @endcode
   *
   * This cannot be implemented in Mutex in a locked() method since this
   * would lead to race conditions in many situations.
   *
   * @return true, if the mutex could be locked, false otherwise.
   */
  bool tryLock();

  /** Unlock the mutex.
   */
  void unlock();

 private:
  MutexData *mutex_data;
};


#endif
