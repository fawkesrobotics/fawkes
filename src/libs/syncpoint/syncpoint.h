/***************************************************************************
 *  syncpoint.h - Fawkes SyncPoint
 *
 *  Created: Thu Jan 09 12:22:03 2014
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

#ifndef __SYNCPOINT_SYNCPOINT_H_
#define __SYNCPOINT_SYNCPOINT_H_

#include <interface/interface.h>
#include <syncpoint/syncpoint_call.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#include <utils/time/time.h>

#include <core/utils/refptr.h>
#include <core/utils/circular_buffer.h>

#include <logging/multi.h>

#include <set>
#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SyncPointManager;
class SyncPoint;

class SyncPointSetLessThan {
  public:
    bool operator()(const RefPtr<SyncPoint> sp1, const RefPtr<SyncPoint> sp2) const;
};


class SyncPoint
{
  public:
    /** Type to define when a thread wakes up after waiting for a SyncPoint.
     * A thread can be either wake up if ANY other thread emits the SyncPoint,
     * or if ALL registered threads emit the SyncPoint.
     */
    typedef enum {
      WAIT_FOR_ONE,
      WAIT_FOR_ALL,
      NONE
    } WakeupType;

    SyncPoint(std::string identifier, MultiLogger *logger,
      uint max_waittime_sec = 0, uint max_waittime_nsec = 0);
    virtual ~SyncPoint();

    /** send a signal to all waiting threads */
    virtual void emit(const std::string & component);

    /** wait for the sync point to be emitted by any other component */
    virtual void wait(const std::string & component, WakeupType = WAIT_FOR_ONE,
      uint wait_sec = 0, uint wait_nsec = 0);
    /** abort waiting */
    virtual void unwait(const std::string & component);
    virtual void wait_for_one(const std::string & component);
    virtual void wait_for_all(const std::string & component);
    /** wait for the sync point, but abort after given time */
    virtual void reltime_wait_for_one(const std::string & component,
      uint wait_sec, uint wait_nsec);
    virtual void reltime_wait_for_all(const std::string & component,
      uint wait_sec, uint wait_nsec);

    /** register as emitter */
    virtual void register_emitter(const std::string & component);

    /** unregister as emitter */
    virtual void unregister_emitter(const std::string & component, bool emit_if_pending = true);
    bool is_emitter(const std::string & component) const;
    bool is_watcher(const std::string & component) const;

    void lock_until_next_wait(const std::string & component);

    std::string get_identifier() const;
    bool operator==(const SyncPoint & other) const;
    bool operator==(const std::string & other) const;
    bool operator<(const SyncPoint & other) const;

    std::set<std::string> get_watchers() const;
    std::multiset<std::string> get_emitters() const;
    CircularBuffer<SyncPointCall> get_wait_calls(WakeupType type = WAIT_FOR_ONE) const;
    CircularBuffer<SyncPointCall> get_emit_calls() const;
    bool watcher_is_waiting(std::string watcher, WakeupType type) const;


    /**
     * allow Syncpoint Manager to edit
     */
    friend class SyncPointManager;

  protected:
    std::pair<std::set<std::string>::iterator,bool> add_watcher(std::string watcher);
    /** send a signal to all waiting threads */
    virtual void emit(const std::string & component, bool remove_from_pending);

  protected:
    /** The unique identifier of the SyncPoint */
    const std::string identifier_;
    /** Set of all components which use this SyncPoint */
    std::set<std::string> watchers_;
    /** Set of all components which are currently waiting for a single emitter */
    std::set<std::string> watchers_wait_for_one_;
    /** Set of all components which are currently waiting on the barrier */
    std::set<std::string> watchers_wait_for_all_;

    /** A buffer of the most recent emit calls. */
    CircularBuffer<SyncPointCall> emit_calls_;
    /** A buffer of the most recent wait calls of type WAIT_FOR_ONE. */
    CircularBuffer<SyncPointCall> wait_for_one_calls_;
    /** A buffer of the most recent wait calls of type WAIT_FOR_ALL. */
    CircularBuffer<SyncPointCall> wait_for_all_calls_;
    /** Time when this SyncPoint was created */
    const Time creation_time_;

    /** Mutex used to protect all member variables */
    Mutex *mutex_;
    /** Mutex used to allow lock_until_next_wait */
    Mutex *mutex_next_wait_;
    /** Mutex used for cond_wait_for_one_ */
    Mutex *mutex_wait_for_one_;
    /** WaitCondition which is used for wait_for_one() */
    WaitCondition *cond_wait_for_one_;
    /** Mutex used for cond_wait_for_all_ */
    Mutex *mutex_wait_for_all_;
    /** WaitCondition which is used for wait_for_all() */
    WaitCondition *cond_wait_for_all_;
    /** true if the wait for all timer is running */
    bool wait_for_all_timer_running_;
    /** the component that started the wait-for-all timer */
    std::string wait_for_all_timer_owner_;
    /** maximum waiting time in secs */
    uint max_waittime_sec_;
    /** maximum waiting time in nsecs */
    uint max_waittime_nsec_;

    /** Logger */
    MultiLogger *logger_;

  private:
    void reset_emitters();
    bool is_pending(std::string component);
    void handle_default(std::string component, WakeupType type);
    void cleanup();

  private:
    /** The predecessor SyncPoint, which is the SyncPoint one level up
     *  e.g. "/test/sp" -> "/test"
     */
    RefPtr<SyncPoint> predecessor_;

    /** all successors */
    std::set<RefPtr<SyncPoint>, SyncPointSetLessThan > successors_;

    std::multiset<std::string> emitters_;
    std::multiset<std::string> pending_emitters_;

    std::set<std::string> bad_components_;

    std::string emit_locker_;

    Time last_emitter_reset_;
};

} // end namespace fawkes

#endif
