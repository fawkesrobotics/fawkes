
/***************************************************************************
 *  wait_condition.h - condition variable implementation
 *
 *  Generated: Thu Sep 14 21:34:58 2006
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

#ifndef __CORE_THREADING_WAIT_CONDITION_H_
#define __CORE_THREADING_WAIT_CONDITION_H_

#include <core/threading/mutex.h>

// avoid more headers with these declarations and macros
class WaitConditionData;

/** Wait until a given condition holds.
 * Consider two values x and y and you want to wait until they are equal.
 * For instance there may be a thread counting up after he has finished one
 * particular job before he goes to handle the next one. After 10 threads you
 * want to send out the produced entities in one batch run. So the sending
 * thread has to wait for the producing thread until 10 packages have been
 * produced. Simplified this could be implemented as
 *
 * @code
 * virtual void run()
 * {
 *   forever {
 *     mutex->lock();
 *     while (count != 10) {
 *       wait_condition->wait(mutex);
 *     }
 *   }
 * }
 * @endcode
 *
 * The other thread will wake up this waiting thread after each produced
 * package (the thread does not have to know after how many packages they are
 * sent out). The code could look like this:
 *
 * @code
 * virtual void run()
 * {
 *   forever {
 *     produce_package();
 *     wait_condition->wakeOne();
 *   }
 * }
 * @endcode
 *
 * @see example_waitcond_serialize.cpp
 *
 * @author Tim Niemueller
 */
class WaitCondition {

 public:
  /** Constructor
   */
  WaitCondition();

  /** Destructor
   */
  ~WaitCondition();

  /** Releases the given mutex and waits for the condition. The mutex must
   * be locked before calling this method. Otherwise it will return immediately
   * with value false.
   * @param mutex the mutex to unlock
   * @param timeout_sec Optional timeout in seconds that we will wait at most before
   * we go on. If timeout_sec and timeout_nanosec equal zero waits forever.
   * @param timeout_nanosec Optional timeout in nanoseconds that we will wait at most before
   * we go on. If timeout_sec and timeout_nanosec equal zero waits forever.
   * @return true, if the thread was woken up by another thread calling wakeOne() or
   * wakeAll(). False, if the mutex was unlocked or the timeout has been reached waiting
   * for the condition.
   */
  bool wait(Mutex *mutex, unsigned int timeout_sec = 0, unsigned int timeout_nanosec = 0);

  /** Wake another thread waiting for this condition.
   * This wakes up any thread waiting for the condition, not a particular one. No guarantee
   * is given about the order of the woken up threads.
   */
  void wakeOne();

  /** Wake up all waiting threads.
   * This wakes up all threads waiting for this condition.
   */
  void wakeAll();

 private:
  WaitConditionData *cond_data;

};




#endif
