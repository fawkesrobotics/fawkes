/***************************************************************************
 *  syncpoint.cpp - Fawkes SyncPoint
 *
 *  Created: Thu Jan 09 12:35:57 2014
 *  Copyright  2014-2018  Till Hofmann
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
 * @param logger The logger to use for error messages.
 * @param max_waittime_sec the maximum number of seconds to wait until a timeout
 * is triggered
 * @param max_waittime_nsec the maximum number of nanoseconds to wait until a
 * timeout is triggered
 */
SyncPoint::SyncPoint(string identifier, MultiLogger *logger,
  uint max_waittime_sec /* = 0 */, uint max_waittime_nsec /* = 0 */)
    : identifier_(identifier),
      emit_calls_(CircularBuffer<SyncPointCall>(1000)),
      wait_for_one_calls_(CircularBuffer<SyncPointCall>(1000)),
      wait_for_all_calls_(CircularBuffer<SyncPointCall>(1000)),
      creation_time_(Time()),
      mutex_(new Mutex()),
      mutex_next_wait_(new Mutex()),
      mutex_wait_for_one_(new Mutex()),
      cond_wait_for_one_(new WaitCondition(mutex_wait_for_one_)),
      mutex_wait_for_all_(new Mutex()),
      cond_wait_for_all_(new WaitCondition(mutex_wait_for_all_)),
      wait_for_all_timer_running_(false),
      max_waittime_sec_(max_waittime_sec),
      max_waittime_nsec_(max_waittime_nsec),
      logger_(logger),
      last_emitter_reset_(Time(0l))
{
  if (identifier.empty()) {
    cleanup();
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
  if (identifier.compare(0,1,"/")) {
    cleanup();
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
  // check if last charater is '/'
  // The identifier may only end in '/' if '/' is the complete identifier.
  // '/' is allowed, '/some/' is not allowed
  if (identifier != "/" && !identifier.compare(identifier.size() - 1, 1, "/")) {
    cleanup();
    throw SyncPointInvalidIdentifierException(identifier.c_str());
  }
}

SyncPoint::~SyncPoint()
{
  cleanup();
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
  emit(component, true);
}

/** Wake up all components which are waiting for this SyncPoint
 * @param component The identifier of the component emitting the SyncPoint
 * @param remove_from_pending if set to true, the component will be removed
 *        from the pending emitters for this syncpoint
 */
void
SyncPoint::emit(const std::string & component, bool remove_from_pending)
{
  mutex_next_wait_->stopby();
  MutexLocker ml(mutex_);
  if (!watchers_.count(component)) {
    throw SyncPointNonWatcherCalledEmitException(component.c_str(),
        get_identifier().c_str());
  }

  // unlock all wait_for_one waiters
  watchers_wait_for_one_.clear();
  mutex_wait_for_one_->lock();
  cond_wait_for_one_->wake_all();
  mutex_wait_for_one_->unlock();


  if (!emitters_.count(component)) {
    throw SyncPointNonEmitterCalledEmitException(component.c_str(),
      get_identifier().c_str());
  }

  /* 1. remember whether the component was pending; if so, it may be removed
   *    from the pending components of the predecessor. Otherwise, it should
   *    not be removed
   * 2. only erase the component once; it may be registered multiple times
   */
  bool pred_remove_from_pending = false;
  if (remove_from_pending) {
    multiset<string>::iterator it_pending = pending_emitters_.find(component);
    if (it_pending != pending_emitters_.end()) {
      pending_emitters_.erase(it_pending);
      if (predecessor_) {
        if (last_emitter_reset_ <= predecessor_->last_emitter_reset_) {
          pred_remove_from_pending = true;
        }
      }

      // unlock all wait_for_all waiters if all pending emitters have emitted
      if (pending_emitters_.empty()) {
        watchers_wait_for_all_.clear();
        mutex_wait_for_all_->lock();
        cond_wait_for_all_->wake_all();
        mutex_wait_for_all_->unlock();
        reset_emitters();
      }
    }
  }

  emit_calls_.push_back(SyncPointCall(component));

  if (predecessor_) {
    predecessor_->emit(component, pred_remove_from_pending);
  }
}

/** Wait until SyncPoint is emitted.
 * Either wait until a single emitter has emitted the SyncPoint, or wait
 * until all registered emitters have emitted the SyncPoint.
 * If wait_sec != 0 or wait_nsec !=0, then only wait for
 * wait_sec + wait_nsec*10^-9 seconds and set the SyncPoint's maximum waiting
 * time to the specified time (i.e., on any subsequent wait calls, wait for
 * the specified time until a timeout is triggered).
 * If the maximal wait time has been exceeded, a warning is shown and the
 * SyncPoint is released.
 * If the WakeupType is WAIT_FOR_ALL, then the time limit is only used if there
 * is currently no other component waiting in WAIT_FOR_ALL mode. If there is
 * already a component waiting, that component's wait_time is used to compute
 * the timeout. This ensures that in case a timeout occurs, all waiting
 * components in WAIT_FOR_ALL mode are released simultaneously. Components in
 * WAIT_FOR_ONE mode are treated separately and have their own timeouts.
 * @param component The identifier of the component waiting for the SyncPoint
 * @param type the wakeup type. If this is set to WAIT_FOR_ONE, wait returns
 * when a single emitter has emitted the SyncPoint. If set to WAIT_FOR_ALL, wait
 * until all registered emitters have emitted the SyncPoint.
 * @param wait_sec number of seconds to wait for the SyncPoint
 * @param wait_nsec number of nanoseconds to wait for the SyncPoint
 * @see SyncPoint::WakeupType
 */
void
SyncPoint::wait(const std::string & component,
  WakeupType type /* = WAIT_FOR_ONE */, uint wait_sec /* = 0 */,
  uint wait_nsec /* = 0 */)
{

  MutexLocker ml(mutex_);

  std::set<std::string> *watchers;
  WaitCondition *cond;
  CircularBuffer<SyncPointCall> *calls;
  Mutex *mutex_cond;
  bool *timer_running;
  string *timer_owner;
  // set watchers, cond and calls depending of the Wakeup type
  if (type == WAIT_FOR_ONE) {
    watchers = &watchers_wait_for_one_;
    cond = cond_wait_for_one_;
    mutex_cond = mutex_wait_for_one_;
    calls = &wait_for_one_calls_;
    timer_running = NULL;
  } else if (type == WAIT_FOR_ALL) {
    watchers = &watchers_wait_for_all_;
    cond = cond_wait_for_all_;
    mutex_cond = mutex_wait_for_all_;
    timer_running = &wait_for_all_timer_running_;
    timer_owner = &wait_for_all_timer_owner_;
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

  /* Check if emitters are currently waiting for this component.
   * If so, wake them up *after* locking the WaitCondition's mutex.
   * Only this way we can guarantee that this component will certainly call
   * wait before the emitters emit
   */
  Time start;
  mutex_cond->lock();
  if (emit_locker_ == component) {
    mutex_next_wait_->unlock();
    emit_locker_ = "";
  }
  if (need_to_wait) {
    if (type == WAIT_FOR_ONE) {
      ml.unlock();
      bool timeout;
      pthread_cleanup_push(cleanup_mutex, mutex_cond);
      timeout = !cond->reltimed_wait(wait_sec, wait_nsec);
      pthread_cleanup_pop(1);
      if (timeout) {
        ml.relock();
        handle_default(component, type);
        ml.unlock();
      }
    } else {
      if (*timer_running) {
        ml.unlock();
        pthread_cleanup_push(cleanup_mutex, mutex_cond);
        cond->wait();
        pthread_cleanup_pop(1);
      } else {
        *timer_running = true;
        *timer_owner = component;
        if (wait_sec != 0 || wait_nsec != 0) {
          max_waittime_sec_ = wait_sec;
          max_waittime_nsec_ = wait_nsec;
        }
        ml.unlock();
        bool timeout;
        pthread_cleanup_push(cleanup_mutex, mutex_cond);
        timeout = !cond->reltimed_wait(max_waittime_sec_, max_waittime_nsec_);
        pthread_cleanup_pop(1);
        ml.relock();
        *timer_running = false;
        if (timeout) {
          // wait failed, handle default
          handle_default(component, type);
          mutex_cond->lock();
          cond->wake_all();
          mutex_cond->unlock();
        }
        ml.unlock();
      }
    }
  } else {
    ml.unlock();
    mutex_cond->unlock();
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

/** Wait for a single emitter for the given time.
 * @param component The identifier of the calling component.
 * @param wait_sec number of seconds to wait
 * @param wait_nsec number of nanoseconds to wait additionally to wait_sec
 */
void
SyncPoint::reltime_wait_for_one(const string & component, uint wait_sec,
  uint wait_nsec)
{
  wait(component, SyncPoint::WAIT_FOR_ONE, wait_sec, wait_nsec);
}

/** Wait for all registered emitters for the given time.
 * @param component The identifier of the calling component.
 * @param wait_sec number of seconds to wait
 * @param wait_nsec number of nanoseconds to wait additionally to wait_sec
 */
void
SyncPoint::reltime_wait_for_all(const string & component, uint wait_sec,
  uint wait_nsec)
{
  wait(component, SyncPoint::WAIT_FOR_ALL, wait_sec, wait_nsec);
}

/** Do not wait for the SyncPoint any longer.
 *  Removes the component from the list of waiters. If the given component is
 *  not waiting, do nothing.
 *  @param component the component to remove from the waiters
 */
void
SyncPoint::unwait(const string & component)
{
  MutexLocker ml(mutex_);
  watchers_wait_for_one_.erase(component);
  watchers_wait_for_all_.erase(component);
  if (wait_for_all_timer_owner_ == component) {
    // TODO: this lets the other waiting components wait indefinitely, even on
    // a timed wait.
    wait_for_all_timer_running_ = false;
  }
}


/** Lock the SyncPoint for emitters until the specified component does the next
 *  wait() call. This forces an emitter of this SyncPoint to wait during the
 *  emit until the waiter calls wait(). This is useful if you want to guarantee
 *  that the waiter does not call wait() immediately after the emitter has
 *  called emit().
 *  @param component the component locking the SyncPoint
 */
void
SyncPoint::lock_until_next_wait(const string & component)
{
  MutexLocker ml(mutex_);
  if (mutex_next_wait_->try_lock()) {
    emit_locker_ = component;
  } else {
    logger_->log_warn("SyncPoints", "%s tried to call lock_until_next_wait, "
        "but another component already did the same. Ignoring.",
        component.c_str());
  }
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

/** Unregister an emitter. This removes the component from the syncpoint, thus
 *  other components will not wait for it anymore.
 *  @param component The identifier of the component which is unregistered.
 *  @param emit_if_pending if this is set to true and the component is a
 *         pending emitter, emit the syncpoint before releasing it.
 */
void
SyncPoint::unregister_emitter(const string & component, bool emit_if_pending) {
  // TODO should this throw if the calling component is not registered?
  multiset<string>::iterator it_emitter = emitters_.find(component);
  if (it_emitter == emitters_.end()) {
	  // component is not an emitter
	  return;
  }
  MutexLocker ml(mutex_);
  if (emit_if_pending && is_pending(component)) {
    ml.unlock();
    emit(component);
    ml.relock();
  }

  // erase a single element from the set of emitters
  emitters_.erase(it_emitter);
  if (predecessor_) {
    // never emit the predecessor if it's pending; it is already emitted above
    predecessor_->unregister_emitter(component, false);
  }
}

/** Check if the given component is an emitter.
 *  @param component The name of the component.
 *  @return True iff the given component is an emitter of this syncpoint.
 */
bool
SyncPoint::is_emitter(const string & component) const {
  MutexLocker ml(mutex_);
	return emitters_.count(component) > 0;
}

/** Check if the given component is a watch.
 *  @param component The name of the component.
 *  @return True iff the given component is a watcher.
 */
bool
SyncPoint::is_watcher(const string & component) const {
  MutexLocker ml(mutex_);
	return watchers_.count(component) > 0;
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
 * @return a copy of the set of registered emitters
 */
multiset<string>
SyncPoint::get_emitters() const
{
  return emitters_;
}

/**
 * @return a copy of the emit call buffer
 */
CircularBuffer<SyncPointCall>
SyncPoint::get_emit_calls() const {
  MutexLocker ml(mutex_);
  return emit_calls_;
}

/**
 * Check if the given waiter is currently waiting with the given type
 * @param watcher the string identifier of the watcher to check
 * @param type the type of call to check
 * @return true if the waiter is currently waiting
 */
bool
SyncPoint::watcher_is_waiting(std::string watcher, WakeupType type) const
{
  switch (type) {
    case SyncPoint::WAIT_FOR_ONE:
      return watchers_wait_for_one_.count(watcher);
    case SyncPoint::WAIT_FOR_ALL:
      return watchers_wait_for_all_.count(watcher);
    default:
      throw Exception("Unknown watch type %u for syncpoint %s",
                      type, identifier_);
  }
}

void
SyncPoint::reset_emitters() {
  last_emitter_reset_ = Time();
  pending_emitters_ = emitters_;
}

bool
SyncPoint::is_pending(string component) {
  return pending_emitters_.count(component) > 0;
}

void
SyncPoint::handle_default(string component, WakeupType type)
{
  logger_->log_warn(component.c_str(),
      "Thread time limit exceeded while waiting for syncpoint '%s'. "
      "Time limit: %f sec.",
      get_identifier().c_str(),
      max_waittime_sec_ + static_cast<float>(max_waittime_nsec_)/1000000000.f);
  bad_components_.insert(pending_emitters_.begin(), pending_emitters_.end());
  if (bad_components_.size() > 1) {
    string bad_components_string = "";
    for (set<string>::const_iterator it = bad_components_.begin();
        it != bad_components_.end(); it++) {
      bad_components_string += " " + *it;
    }
    logger_->log_warn(component.c_str(), "bad components:%s",
        bad_components_string.c_str());
  }
  else if (bad_components_.size() == 1) {
    logger_->log_warn(component.c_str(), "bad component: %s",
        bad_components_.begin()->c_str());
  }
  else if (type == SyncPoint::WAIT_FOR_ALL) {
    throw Exception("SyncPoints: component %s defaulted, "
        "but there is no pending emitter. This is probably a bug.",
        component.c_str());
  }

  watchers_wait_for_all_.erase(component);
  watchers_wait_for_one_.erase(component);
}

void
SyncPoint::cleanup()
{
  delete cond_wait_for_one_;
  delete mutex_wait_for_one_;
  delete cond_wait_for_all_;
  delete mutex_wait_for_all_;
  delete mutex_next_wait_;
  delete mutex_;
}
} // namespace fawkes
