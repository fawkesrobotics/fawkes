
/***************************************************************************
 *  wait_condition.cpp - condition variable implementation
 *
 *  Created: Thu Sep 14 21:43:30 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_data.h>
#include <core/threading/wait_condition.h>

#include <chrono>
#include <condition_variable>

namespace fawkes {

/** @class WaitCondition <core/threading/wait_condition.h>
 * Wait until a given condition holds.
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
 *       wait_condition->wait();
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
 *     wait_condition->wake_one();
 *   }
 * }
 * @endcode
 *
 * The WaitCondition can operate in two principal modes, either with an internal
 * or with an external Mutex. If no mutex is passed to the constructor an
 * internal mutex is created and used. If a mutex is passed this instance is used,
 * but ownership is not claimed and you have to delete it manually. Additionally,
 * for external mutexes they are <i>never</i> locked by the wait condition. For
 * external mutexes you get all the freedom, but also have the duty to ensure
 * proper locking from the outside! This applies to wait and wake methods.
 *
 * @ingroup Threading
 * @ingroup FCL
 * @see Mutex
 * @see qa_waitcond_serialize.cpp
 * @see qa_waitcond.cpp
 *
 * @author Tim Niemueller
 *
 */

/** Constructor.
 * @param mutex the mutex used for this wait condition. If none is given, an
 * internal mutex will be created and used.
 */
WaitCondition::WaitCondition(Mutex *mutex)
{
	if (mutex) {
		mutex_     = mutex;
		own_mutex_ = false;
	} else {
		mutex_     = new Mutex();
		own_mutex_ = true;
	}
}

/** Destructor. */
WaitCondition::~WaitCondition()
{
	if (own_mutex_) {
		delete mutex_;
	}
}

/** Wait for the condition forever.
 * This waits forever until a wakup signal is received by another thread calling
 * wake_all() or wake_one(). If an external mutex is used it must be locked or
 * before calling wait() or the result is undefined. After the method returns
 * the mutex is locked again.
 */
void
WaitCondition::wait()
{
	auto lock = get_lock();
	cond_var_.wait(lock);
}

/** Wait with absolute timeout.
 * This waits for the given mutex until either a wakup signal is received or
 * the timeout has passed. The timeout has to be given in absolute system time,
 * a simulated clock source cannot be used.
 * @param sec Seconds of absolute time since the epoch (value compatible to
 * timeval tv_sec part is sufficient).
 * @param nanosec Nanoseconds part of the absolute timeout. Added to the seconds
 * part.
 * @return true, if the thread was woken up by another thread calling
 * wake_one() or wake_all(), false otherwise if the timeout has been reached
 * @exception Exception thrown if another error occurs for the POSIX wait condition
 */
bool
WaitCondition::abstimed_wait(long int sec, long int nanosec)
{
	auto lock = get_lock();
	return cond_var_.wait_until(
	         lock,
	         std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
	           std::chrono::seconds(sec) + std::chrono::nanoseconds(nanosec)))
	       == std::cv_status::no_timeout;
}

/** Wait with relative timeout.
 * This waits for the given mutex until either a wakup signal is received or
 * the timeout has passed. The timeout has to be given in relative system time.
 * It is added to the current time and is then used similar to abstime_wait().
 * A timeout of (0,0) will cause this method to wait forever, similar to wait().
 * @param sec Number of seconds to wait
 * @param nanosec Number of nanoseconds to wait, added to seconds value
 * @return true, if the thread was woken up by another thread calling
 * wake_one() or wake_all(), false otherwise if the timeout has been reached
 * @exception Exception thrown if another error occurs for the POSIX wait condition
 */
bool
WaitCondition::reltimed_wait(unsigned int sec, unsigned int nanosec)
{
	if (!(sec || nanosec)) {
		wait();
		return true;
	}

	auto lock = get_lock();
	return cond_var_.wait_for(lock, std::chrono::seconds(sec) + std::chrono::nanoseconds(nanosec))
	       == std::cv_status::no_timeout;
}

/** Wake another thread waiting for this condition.
 * This wakes up any thread waiting for the condition, not a particular one.
 * No guarantee is given about the order of the woken up threads.
 * Note: If the internal mutex is used for this wait/wakeup cycle, the lock
 * to this mutex will be acquired during the wakeup, to ensure that all waiting
 * threads are woken up, even if a call to wait() and wake_one() overlapped.
 * If however, an external Mutex is used, you must ensure by yourself that it
 * is properly locked during the wakeup to ensure this.
 */
void
WaitCondition::wake_one()
{
	if (own_mutex_) { // it's our internal mutex, lock!
		mutex_->lock();
	}
	cond_var_.notify_one();
	if (own_mutex_) {
		mutex_->unlock();
	}
}

/** Wake up all waiting threads.
 * This wakes up all threads waiting for this condition.
 * Note: If the internal mutex is used for this wait/wakeup cycle, the lock
 * to this mutex will be acquired during the wakeup, to ensure that all waiting
 * threads are woken up, even if a call to wait() and wake_one() overlapped.
 * If however, an external Mutex is used, you must ensure by yourself that it
 * is properly locked during the wakeup to ensure this.
 */
void
WaitCondition::wake_all()
{
	if (own_mutex_) { // it's our internal mutex, lock!
		mutex_->lock();
	}
	cond_var_.notify_all();
	if (own_mutex_) {
		mutex_->unlock();
	}
}

std::unique_lock<std::mutex>
WaitCondition::get_lock()
{
	std::unique_lock<std::mutex> lock;
	if (own_mutex_) {
		lock = std::unique_lock<std::mutex>(mutex_->get_raw_mutex());
	} else {
		// We assume the caller has already locked the mutex, thuse we defer locking.
		lock = std::unique_lock<std::mutex>(mutex_->get_raw_mutex(), std::defer_lock_t());
	}
	return lock;
}

} // end namespace fawkes
