
/***************************************************************************
 *  barrier.cpp - Barrier
 *
 *  Generated: Thu Sep 15 00:33:13 2006
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

#include <core/threading/barrier.h>

#include <pthread.h>

/// @cond INTERNALS
class BarrierData
{
 public:
  pthread_barrier_t barrier;
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
 * @ingroup Threading
 * @author Tim Niemueller
 */


/** Constructor
 * @param count the number of threads to wait for
 */
Barrier::Barrier(unsigned int count)
{
  barrier_data = new BarrierData();
  pthread_barrier_init( &(barrier_data->barrier), NULL, count );
}


/** Destructor */
Barrier::~Barrier()
{
  pthread_barrier_destroy( &(barrier_data->barrier) );
  delete barrier_data;
}


/** Wait for other threads.
 * This method will block until as many threads have called wait as you have
 * given count to the constructor.
 */
void
Barrier::wait()
{
  pthread_barrier_wait( &(barrier_data->barrier) );
}

