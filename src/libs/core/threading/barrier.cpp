
/***************************************************************************
 *  barrier.cpp - Barrier
 *
 *  Created: Thu Sep 15 00:33:13 2006
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

#include <core/threading/barrier.h>
#include <core/exception.h>

#include <pthread.h>
#include <unistd.h>

// cf. http://people.redhat.com/drepper/posix-option-groups.html
#if defined(_POSIX_BARRIERS) && (_POSIX_BARRIERS - 200112L) >= 0
#  define USE_POSIX_BARRIERS
#else
#  undef USE_POSIX_BARRIERS
#  include <core/threading/mutex.h>
#  include <core/threading/wait_condition.h>
#endif

namespace fawkes {


/// @cond INTERNALS
class BarrierData
{
 public:
#ifdef USE_POSIX_BARRIERS
  pthread_barrier_t barrier;
#else
  BarrierData() : threads_left(0), mutex(), waitcond(&mutex) {}

  unsigned int  threads_left;
  Mutex         mutex;
  WaitCondition waitcond;
#endif
};
/// @endcond


/** @class Barrier core/threading/barrier.h
 * A barrier is a synchronization tool which blocks until a given number of
 * threads have reached the barrier.
 *
 * Consider you have three threads all doing work. In the end you have to merge
 * their results. For this you need a point where all threads are finished. Of
 * course you don't want another thread waiting and periodically checking if
 * the other threads are finished, that will loose time - processing time and
 * the time that passed between the threads being finished and the time when
 * the controller checked again.
 *
 * For this problem we have a barrier. You initialize the barrier with the
 * number of threads to wait for and then wait in all threads at a specific
 * point. After this one of the threads can merge the result (and the others
 * may wait for another "go-on barrier".
 *
 * The code in the thread would be
 * @code
 * virtual void run()
 * {
 *   forever {
 *     do_something();
 *     result_barrier->wait();
 *     if ( master_thread ) {
 *       merge_results();
 *     }
 *     goon_barrier->wait();
 *   }
 * }
 * @endcode
 *
 * The threads would all get a reference to the same barriers and one would
 * have master thread to be true.
 *
 * After the barrier has been passed (count threads waited) the barrier is
 * reset automatically and can be re-used with the same amount of barrier
 * waiters.
 *
 * The implementation of Barrier takes into account that PThread barriers are
 * optional in POSIX. If barriers are not available they are emulated using a
 * wait condition. Note however that on systems that do have both, barriers and
 * wait conditions it has been observed that in a typical barrier scenario the
 * POSIX barriers perform much better in many situations than a wait condition
 * (processes tend to be rescheduled immediately if a barrier is reached, while
 * with a wait condition they are rescheduled with lower priority and thus they
 * delay may increase on a loaded system). Because of this on systems without
 * real POSIX barriers the performance may be not as good as is expected.
 *
 * @ingroup Threading
 * @author Tim Niemueller
 */


/** Constructor
 * @param count the number of threads to wait for
 */
Barrier::Barrier(unsigned int count)
{
  _count = count;
  if ( _count == 0 ) {
    throw Exception("Barrier count must be at least 1");
  }
  barrier_data = new BarrierData();
#ifdef USE_POSIX_BARRIERS
  pthread_barrier_init( &(barrier_data->barrier), NULL, _count );
#else
  barrier_data->threads_left = _count;
#endif
}


/** Protected Constructor.
 * Does not initialize any internal structures other than setting them to 0.
 */
Barrier::Barrier()
{
  _count = 0;
  barrier_data = NULL;
}


/** Destructor */
Barrier::~Barrier()
{
  if (barrier_data) {
#ifdef USE_POSIX_BARRIERS
    pthread_barrier_destroy( &(barrier_data->barrier) );
#endif
    delete barrier_data;
  }
}


/** Wait for other threads.
 * This method will block until as many threads have called wait as you have
 * given count to the constructor.
 */
void
Barrier::wait()
{
#ifdef USE_POSIX_BARRIERS
  pthread_barrier_wait( &(barrier_data->barrier) );
#else
  barrier_data->mutex.lock();

  if ( --barrier_data->threads_left == 0 ) {
    barrier_data->threads_left = _count;
    barrier_data->mutex.unlock();
    barrier_data->waitcond.wake_all();
  } else {
    barrier_data->waitcond.wait();
    barrier_data->mutex.unlock();
  }

#endif
}


/** Get number of threads this barrier will wait for.
 * @return number of threads this barrier will wait for.
 */
unsigned int
Barrier::count()
{
  return _count;
}


} // end namespace fawkes
