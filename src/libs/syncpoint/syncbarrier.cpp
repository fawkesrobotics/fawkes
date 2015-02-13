/***************************************************************************
 *  syncbarrier.cpp - Fawkes SyncBarrier
 *
 *  Created: Fri Jan 16 14:02:42 2015
 *  Copyright  2015  Till Hofmann
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

#include <syncpoint/syncpoint.h>
#include <syncpoint/syncbarrier.h>
#include <syncpoint/exceptions.h>

#include <utils/time/time.h>

#include <core/threading/mutex_locker.h>

#include <string.h>

using namespace std;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class SyncBarrier <syncpoint/syncbarrier.h>
 * The SyncBarrier class.
 * This class is used for dynamic synchronization of threads which depend
 * on each other, e.g. threads which are part of a processing chain.
 * Similar to a SyncPoint, components use a SyncBarrier either as an emitter or
 * as a waiter. Emitters first have to register with the barrier in order to be
 * allowed to emit it. With SyncBarriers, a waiter blocks until all registered
 * emitters have emitted the barrier. A SyncBarrier runs in iterations: Once an
 * emitter has emitted the barrier, it is no longer considered to be pending,
 * i.e. a waiter will not wait for the emitter until the SyncBarrier is reset.
 * A reset causes all registered emitters to be pending again.
 *
 * As an example, thread E and F generate data which is needed by thread W.
 * Therefore, all three threads share a SyncBarrier.
 * Thread W wait()s for the SyncBarrier to be emitted.
 * Once threads E and F are done, they emit() the SyncBarrier. If the barrier
 * has been emitted by both threads, thread W unblocks and continues to process
 * the data.
 *
 * @author Till Hofmann
 * @see SyncPointManager
 */

/** Constructor.
 * @param identifier The identifier of the SyncBarrier. This must be in absolute
 * path style, e.g. '/some/barrier'.
 */
SyncBarrier::SyncBarrier(string identifier) :
    SyncPoint(identifier)
{

}


/** Emit the SyncBarrier.
 *  If all registered emitters have emitted the barriers, all waiters are woken up.
 *  @param component The identifier of the component emitting the SyncBarrier
 */
void
SyncBarrier::emit(const string & component)
{
  MutexLocker ml(mutex_);
  if (!emitters_.count(component)) {
    throw SyncBarrierNonEmitterCalledEmitException(component.c_str(),
      get_identifier().c_str());
  }

  // we do NOT expect the component to be pending
  // a component may call emit multiple times in a loop
  pending_emitters_.erase(component);
  emit_calls_.push_back(SyncPointCall(component));

  // all emitters have emitted the signal, thus wake all waking components
  if (pending_emitters_.empty()) {
    waiting_watchers_.clear();
    wait_condition_->wake_all();
    reset_emitters_();
  }

  if (predecessor_) {
    predecessor_->emit(component);
  }
}

/** Wait for the SyncBarrier.
 *  If no emitters have registered for the barrier, return immediately.
 *  Otherwise, wait until all registered emitters have emitted the SyncBarrier.
 *  @param component The identifier of the component waiting for the SyncBarrier
 */
void
SyncBarrier::wait(const string & component)
{
  mutex_->lock();
  bool need_to_wait = !emitters_.empty();
  mutex_->unlock();
  if (need_to_wait) {
    SyncPoint::wait(component);
  }
}


/** Register an emitter. A thread can only emit the barrier if it has been
 *  registered.
 *  @param component The identifier of the registering component.
 */
void
SyncBarrier::register_emitter(const string & component)
{
  MutexLocker ml(mutex_);
  if (!emitters_.insert(component).second) {
    throw SyncBarrierMultipleRegisterCallsException(component.c_str(),
      get_identifier().c_str());
  }
  // TODO if a new emitter registers, should it already be part of the barrier
  // in this iteration? If so, we need to insert it in pending_emitters, too
  if (predecessor_) {
    predecessor_->register_emitter(component);
  }
}

/** Unregister an emitter. This removes the component from the barrier, thus
 *  other components will not wait for it anymore.
 *  @param component The identifier of the component which is unregistered.
 */
void
SyncBarrier::unregister_emitter(const string & component) {
  // TODO should this throw if the calling component is not registered?
  MutexLocker ml(mutex_);
  pending_emitters_.erase(component);
  emitters_.erase(component);
  if (predecessor_) {
    predecessor_->unregister_emitter(component);
  }
}

/** Check if the barrier is already open,
 *  i.e. if all registered emitters have already emitted the barrier.
 *  @return true if the SyncBarrier has been emitted.
 */
bool
SyncBarrier::is_emitted() const {
	MutexLocker ml(mutex_);
	return pending_emitters_.empty();
}

/** Reset all emitters. After reset, all registered emitters are again pending.
 *  Any waiter will block until all emitters have emitted the barrier again.
 */
void
SyncBarrier::reset_emitters() {
	MutexLocker ml(mutex_);
	reset_emitters_();
}

/** Helper function. This allows to reset emitters both with mutex_ locked
 *  and unlocked.
 */
void
SyncBarrier::reset_emitters_() {
  pending_emitters_ = emitters_;
}

} // namespace fawkes
