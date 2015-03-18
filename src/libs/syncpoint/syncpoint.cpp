/***************************************************************************
 *  syncpoint.cpp - Fawkes SyncPoint
 *
 *  Created: Thu Jan 09 12:35:57 2014
 *  Copyright  2014  Till Hofmann
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
#include <syncpoint/exceptions.h>

#include <core/threading/mutex_locker.h>
#include <utils/time/time.h>

#include <string.h>

using namespace std;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class SyncPoint <syncpoint/syncpoint.h>
 * The SyncPoint class.
 * This class is used for dynamic synchronization of threads which depend
 * on each other, e.g. threads which are part of a processing chain.
 *
 * As an example, thread E generates data which is needed by thread W.
 * Therefore, both threads share a SyncPoint.
 * Thread W wait()s for the SyncPoint to be emitted.
 * Once thread E is done, it emit()s the SyncPoint, which wakes up thread W.
 *
 * @author Till Hofmann
 * @see SyncPointManager
 */

/** Constructor.
 * @param identifier The identifier of the SyncPoint. This must be in absolute
 * path style, e.g. '/some/syncpoint'.
 */
SyncPoint::SyncPoint(string identifier)
    : identifier_(identifier),
      emit_calls_(CircularBuffer<SyncPointCall>(1000)),
      wait_for_one_calls_(CircularBuffer<SyncPointCall>(1000)),
      wait_for_all_calls_(CircularBuffer<SyncPointCall>(1000)),
      creation_time_(Time()),
      mutex_(new Mutex()),
      cond_wait_for_one_(new WaitCondition()),
      cond_wait_for_all_(new WaitCondition())
{
  if (identifier.empty()) {
    delete cond_wait_for_one_;
    delete cond_wait_for_all_;
    delete mutex_;
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
  if (identifier.compare(0,1,"/")) {
    delete cond_wait_for_one_;
    delete cond_wait_for_all_;
    delete mutex_;
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
  // check if last charater is '/'
  // The identifier may only end in '/' if '/' is the complete identifier.
  // '/' is allowed, '/some/' is not allowed
  if (identifier != "/" && !identifier.compare(identifier.size() - 1, 1, "/")) {
    delete cond_wait_for_one_;
    delete cond_wait_for_all_;
    delete mutex_;
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
}

SyncPoint::~SyncPoint()
{
  delete cond_wait_for_one_;
  delete cond_wait_for_all_;
  delete mutex_;
}

/**
 * @return the identifier of the SyncPoint
 */
string
SyncPoint::get_identifier() const {
  return identifier_;
}

/** EqualOperator.
 * Two SyncPoints are considered equal iff they have the same identifier
 * @param other The other SyncPoint
 * @return true if the identifiers of the SyncPoints are equal
 */
bool
SyncPoint::operator==(const SyncPoint &other) const
{
  return identifier_ == other.get_identifier();
}

/** EqualOperator.
 * A SyncPoint is equal to a given string iff the string is equal to the
 * SyncPoint's identifier.
 * @param other the string to compare
 * @return true if the identifier of the SyncPoint matches the given string
 */
bool
SyncPoint::operator==(const string & other) const
{
  return identifier_ == other;
}

/** LessThan Operator.
 * Compare two SyncPoints using their identifiers.
 * @param other The other SyncPoint
 * @return true if strcmp returns a value < 0 for the identifiers
 */
bool
SyncPoint::operator<(const SyncPoint &other) const
{
  return identifier_ < other.get_identifier();
}

/** Wake up all components which are waiting for this SyncPoint
 * @param component The identifier of the component emitting the SyncPoint
 */
void
SyncPoint::emit(const std::string & component)
{
  MutexLocker ml(mutex_);
  if (!watchers_.count(component)) {
    throw SyncPointNonWatcherCalledEmitException(component.c_str(), get_identifier().c_str());
  }

  // unlock all wait_for_one waiters
  watchers_wait_for_one_.clear();
  cond_wait_for_one_->wake_all();

  // unlock all wait_for_all waiters if all pending emitters have emitted

  if (!emitters_.count(component)) {
    throw SyncPointNonEmitterCalledEmitException(component.c_str(),
      get_identifier().c_str());
  }
  // we do NOT expect the component to be pending
  // a component may call emit multiple times in a loop
  pending_emitters_.erase(component);
  if (pending_emitters_.empty()) {
    // all emitters have emitted the signal, thus wake all waking components
    watchers_wait_for_all_.clear();
    cond_wait_for_all_->wake_all();
    reset_emitters();
  }

  emit_calls_.push_back(SyncPointCall(component));

  ml.unlock();

  if (predecessor_) {
    predecessor_->emit(component);
  }
}

/** Wait until SyncPoint is emitted.
 * Either wait until a single emitter has emitted the SyncPoint, or wait
 * until all registered emitters have emitted the SyncPoint.
 * @param component The identifier of the component waiting for the SyncPoint
 * @param type the wakeup type. If this is set to WAIT_FOR_ONE, wait returns
 * when a single emitter has emitted the SyncPoint. If set to WAIT_FOR_ALL, wait
 * until all registered emitters have emitted the SyncPoint.
 * @see SyncPoint::WakeupType
 */
void
SyncPoint::wait(const std::string & component, WakeupType type /* = WAIT_FOR_ONE */) {
  MutexLocker ml(mutex_);

  std::set<std::string> *watchers;
  WaitCondition *cond;
  CircularBuffer<SyncPointCall> *calls;
  // set watchers, cond and calls depending of the Wakeup type
  if (type == WAIT_FOR_ONE) {
    watchers = &watchers_wait_for_one_;
    cond = cond_wait_for_one_;
    calls = &wait_for_one_calls_;
  } else if (type == WAIT_FOR_ALL) {
    watchers = &watchers_wait_for_all_;
    cond = cond_wait_for_all_;
    calls = &wait_for_all_calls_;
  } else {
    throw SyncPointInvalidTypeException();
  }

  // check if calling component is registered for this SyncPoint
  if (!watchers_.count(component)) {
    throw SyncPointNonWatcherCalledWaitException(component.c_str(), get_identifier().c_str());
  }
  // check if calling component is not already waiting
  if (watchers->count(component)) {
    throw SyncPointMultipleWaitCallsException(component.c_str(), get_identifier().c_str());
  }

  /* if type == WAIT_FOR_ALL but no emitter has registered, we can
   * immediately return
   * if type == WAIT_FOR_ONE, we always wait
   */
  bool need_to_wait = !emitters_.empty() || type == WAIT_FOR_ONE;
  if (need_to_wait) {
    watchers->insert(component);
  }
  ml.unlock();
  Time start;
  if (need_to_wait) {
    cond->wait();
  }
  Time wait_time = Time() - start;
  ml.relock();
  calls->push_back(SyncPointCall(component, start, wait_time));
}

/** Wait for a single emitter.
 * @param component The identifier of the calling component.
 */
void
SyncPoint::wait_for_one(const string & component)
{
  wait(component, WAIT_FOR_ONE);
}

/** Wait for all registered emitters.
 * @param component The identifier of the calling component.
 */
void
SyncPoint::wait_for_all(const string & component)
{
  wait(component, WAIT_FOR_ALL);
}


/** Register an emitter. A thread can only emit the barrier if it has been
 *  registered.
 *  @param component The identifier of the registering component.
 */
void
SyncPoint::register_emitter(const string & component)
{
  MutexLocker ml(mutex_);
  emitters_.insert(component);
  pending_emitters_.insert(component);
  if (predecessor_) {
    predecessor_->register_emitter(component);
  }
}

/** Unregister an emitter. This removes the component from the barrier, thus
 *  other components will not wait for it anymore.
 *  @param component The identifier of the component which is unregistered.
 */
void
SyncPoint::unregister_emitter(const string & component) {
  // TODO should this throw if the calling component is not registered?
  MutexLocker ml(mutex_);
  pending_emitters_.erase(component);
  emitters_.erase(component);
  if (predecessor_) {
    predecessor_->unregister_emitter(component);
  }
}

/** Add a watcher to the watch list
 *  @param watcher the new watcher
 *  @return A pair, of which the first element is an iterator that points
 *           to the possibly inserted element, and the second is a bool
 *           that is true if the element was actually inserted.
 */
pair<set<string>::iterator,bool>
SyncPoint::add_watcher(string watcher)
{
  MutexLocker ml(mutex_);
  return watchers_.insert(watcher);
}

/**
 * @return all watchers of the SyncPoint
 */
std::set<std::string>
SyncPoint::get_watchers() const {
  MutexLocker ml(mutex_);
  return watchers_;
}

/**
 * @return a copy of the wait call buffer with the given type
 * @param type the type of the wait call buffer
 */
CircularBuffer<SyncPointCall>
SyncPoint::get_wait_calls(WakeupType type /* = WAIT_FOR_ONE */) const {
  MutexLocker ml(mutex_);
  if (type == WAIT_FOR_ONE) {
    return wait_for_one_calls_;
  } else if (type == WAIT_FOR_ALL) {
    return wait_for_all_calls_;
  } else {
    throw SyncPointInvalidTypeException();
  }
}


/**
 * @return a copy of the emit call buffer
 */
CircularBuffer<SyncPointCall>
SyncPoint::get_emit_calls() const {
  MutexLocker ml(mutex_);
  return emit_calls_;
}

void
SyncPoint::reset_emitters() {
  pending_emitters_ = emitters_;
}

} // namespace fawkes
