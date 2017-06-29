
/***************************************************************************
 *  aqt_vision_threads.cpp - FireVision Base Vision Camera Data
 *
 *  Created: Mon Sep 24 16:16:25 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "aqt_vision_threads.h"

#include <core/threading/barrier.h>
#include <utils/time/clock.h>
#include <aspect/vision.h>

#include <algorithm>

using namespace fawkes;

/** @class FvAqtVisionThreads "aqt_vision_threads.h"
 * Aquisition-dependant threads.
 * This class is used for managing the vision threads that depend on a
 * given aquisition thread. Used internally in base vision.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param clock Clock for timeout handling, system time is used only.
 */
FvAqtVisionThreads::FvAqtVisionThreads(fawkes::Clock *clock)
{
  this->clock = clock;

  _empty_time = new Time();
  clock->get_systime(_empty_time);

  running_threads_cyclic = new ThreadList();
  running_threads_cont = new ThreadList();
  waiting_threads = new ThreadList();

  cyclic_barrier = new Barrier(1);
}


/** Destructor. */
FvAqtVisionThreads::~FvAqtVisionThreads()
{
  delete _empty_time;
  delete cyclic_barrier;

  delete running_threads_cyclic;
  delete running_threads_cont;
  delete waiting_threads;

}


/** Add a thread in waiting state.
 * The thread is marked as dependant but it is otherwise ignored.
 * @param thread thread to add
 * for access to the YUV422_PLANAR image.
 */
void
FvAqtVisionThreads::add_waiting_thread(Thread *thread)
{
  waiting_threads->push_back_locked(thread);
}


/** Mark the thread as running.
 * @param thread thread to mark as running
 */
void
FvAqtVisionThreads::set_thread_running(Thread *thread)
{
  VisionAspect *vision_thread = dynamic_cast<VisionAspect *>(thread);
  if ( find(waiting_threads->begin(), waiting_threads->end(), thread) != waiting_threads->end()) {
    if ( vision_thread->vision_thread_mode() == VisionAspect::CYCLIC ) {
      running_threads_cyclic->push_back_locked(thread);
      delete cyclic_barrier;
      cyclic_barrier = new Barrier( running_threads_cyclic->size() + 1 );
    } else {
      running_threads_cont->push_back_locked(thread);
    }
    waiting_threads->remove_locked(thread);
  }
}


/** Remove a thread.
 * The thread is removed from all internal structures.
 * @param thread thread to remove
 */
void
FvAqtVisionThreads::remove_thread(Thread *thread)
{
  waiting_threads->remove_locked(thread);
  if ( find(running_threads_cyclic->begin(), running_threads_cyclic->end(), thread) !=
       running_threads_cyclic->end()) {

    running_threads_cyclic->remove_locked(thread);

    delete cyclic_barrier;
    cyclic_barrier = new Barrier( running_threads_cyclic->size() + 1 );
  }
  running_threads_cont->remove_locked(thread);
  if ( empty() ) {
    clock->get_systime(_empty_time);
  }
}


/** Remove waiting thread.
 * @param thread thread to remove from waiting structures.
 */
void
FvAqtVisionThreads::remove_waiting_thread(Thread *thread)
{
  waiting_threads->remove_locked(thread);
  if ( empty() ) {
    clock->get_systime(_empty_time);
  }
}


/** Check if there is at least one cyclic thread.
 * @return true if there is at least one cyclic thread, false otherwise.
 */
bool
FvAqtVisionThreads::has_cyclic_thread()
{
  return ( ! running_threads_cyclic->empty() );
}


/** Check if there is at least one continuous thread.
 * @return true if there is at least one continuous thread, false otherwise.
 */
bool
FvAqtVisionThreads::has_cont_thread()
{
  return ( ! running_threads_cont->empty() );
}


/** Check if the given waiting thread is registered.
 * @param t thread to check for
 * @return true if the given thread is marked as waiting, false otherwise
 */
bool
FvAqtVisionThreads::has_waiting_thread(Thread *t)
{
  return (find(waiting_threads->begin(), waiting_threads->end(), t) != waiting_threads->end());
}


/** Check if there is no thread at all.
 * @return true if there is no thread in any of the internal running or waiting
 * lists, false otherwise
 */
bool
FvAqtVisionThreads::empty()
{
  return ( waiting_threads->empty() &&
	   running_threads_cyclic->empty() &&
	   running_threads_cont->empty() );
}


/** Get the empty time.
 * @return the time in seconds since the last thread has been removed.
 */
float
FvAqtVisionThreads::empty_time()
{
  return clock->elapsed(_empty_time);
}


/** Wakeup and wait for all cyclic threads. */
void
FvAqtVisionThreads::wakeup_and_wait_cyclic_threads()
{
  if ( has_cyclic_thread() ) {
    running_threads_cyclic->wakeup(cyclic_barrier);
    cyclic_barrier->wait();
  }
}


/** Set prepfin hold fo cyclic threads.
 * Sets prepfin hold for cyclice threads and rolls back if it cannot
 * be set for any of the threads.
 * @param hold prepfin hold value
 * @exception Exception thrown if setting fails
 * @see Thread::set_prepfin_hold()
 * @see ThreadList::set_prepfin_hold()
 */
void
FvAqtVisionThreads::set_prepfin_hold(bool hold)
{
  try {
    running_threads_cyclic->set_prepfin_hold(hold);
  } catch (Exception &e) {
    running_threads_cyclic->set_prepfin_hold(false);
    throw;
  }
}
