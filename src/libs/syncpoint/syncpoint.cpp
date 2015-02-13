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
      wait_calls_(CircularBuffer<SyncPointCall>(1000)),
      creation_time_(Time()),
      mutex_(new Mutex()),
      wait_condition_(new WaitCondition())
{
  if (identifier.empty()) {
    delete wait_condition_;
    delete mutex_;
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
  if (identifier.compare(0,1,"/")) {
    delete wait_condition_;
    delete mutex_;
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
  // check if last charater is '/'
  // The identifier may only end in '/' if '/' is the complete identifier.
  // '/' is allowed, '/some/' is not allowed
  if (identifier != "/" && !identifier.compare(identifier.size() - 1, 1, "/")) {
    delete wait_condition_;
    delete mutex_;
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
}

SyncPoint::~SyncPoint()
{
  delete wait_condition_;
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
  mutex_->lock();
  if (!watchers_.count(component)) {
    mutex_->unlock();
    throw SyncPointNonWatcherCalledEmitException(component.c_str(), get_identifier().c_str());
  }
  waiting_watchers_.clear();
  emit_calls_.push_back(SyncPointCall(component));
  mutex_->unlock();
  wait_condition_->wake_all();
  if (predecessor_) {
    predecessor_->emit(component);
  }
}

/** Wait until SyncPoint is emitted
 * @param component The identifier of the component waiting for the SyncPoint
 */
void
SyncPoint::wait(const std::string & component) {
  mutex_->lock();
  // check if calling component is registered for this SyncPoint
  if (!watchers_.count(component)) {
    mutex_->unlock();
    throw SyncPointNonWatcherCalledWaitException(component.c_str(), get_identifier().c_str());
  }
  // check if calling component is not already waiting
  if (waiting_watchers_.count(component)) {
    mutex_->unlock();
    throw SyncPointMultipleWaitCallsException(component.c_str(), get_identifier().c_str());
  }
  waiting_watchers_.insert(component);
  mutex_->unlock();
  Time start;
  wait_condition_->wait();
  Time wait_time = Time() - start;
  mutex_->lock();
  wait_calls_.push_back(SyncPointCall(component, start, wait_time));
  mutex_->unlock();
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
 * @return a copy of the wait call buffer
 */
CircularBuffer<SyncPointCall>
SyncPoint::get_wait_calls() const {
  MutexLocker ml(mutex_);
  return wait_calls_;
}


/**
 * @return a copy of the emit call buffer
 */
CircularBuffer<SyncPointCall>
SyncPoint::get_emit_calls() const {
  MutexLocker ml(mutex_);
  return emit_calls_;
}

} // namespace fawkes
