/***************************************************************************
 *  blackboard_listener_thread.cpp - Convert blackboard events to eclipse terms
 *
 *  Copyright  2017  Victor Mataré
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

#include "blackboard_listener_thread.h"

#include <core/threading/mutex_locker.h>

using namespace fawkes;
using namespace std;

BlackboardListenerThread *BlackboardListenerThread::instance_ = nullptr;

BlackboardListenerThread::BlackboardListenerThread()
  : Thread("ProtoboardBlackboardManager", Thread::OPMODE_WAITFORWAKEUP)
  , BlackBoardInterfaceListener("eclipse-clp")
{
}


/** Get the singleton instance of this thread
 * @return THE instance
 */
BlackboardListenerThread *BlackboardListenerThread::instance()
{
  if (!instance_)
    instance_ = new BlackboardListenerThread();
  return instance_;
}


/** Delete singleton instance, e.g. when unloading the plugin */
void BlackboardListenerThread::cleanup_instance()
{ delete instance_; }


/** Trigger events if an interface matching the pattern is created or destroyed
 * @param type_pattern See BlackBoardInterfaceObserver::bbio_add_observed_create
 * @param id_pattern See BlackBoardInterfaceObserver::bbio_add_observed_create
 */
void BlackboardListenerThread::observe_pattern(
    const char *type_pattern, const char *id_pattern) noexcept
{
  MutexLocker lock(&state_mutex_);
  bbio_add_observed_create(type_pattern, id_pattern);
  bbio_add_observed_destroy(type_pattern, id_pattern);
}


/** Register @param interface for change notifications */
void BlackboardListenerThread::listen_for_change(Interface *interface) noexcept
{
  MutexLocker lock(&state_mutex_);
  bbil_add_data_interface(interface);
}


/** Called by the BlackBoardInterfaceObserver when an interface matching a subscribed pattern is created
 * @param type Interface type name
 * @param id Interface ID
 */
void BlackboardListenerThread::bb_interface_created(const char *type, const char *id) noexcept
{
  MutexLocker lock(&state_mutex_);
  iface_events_.emplace(new BlackboardListenerThread::Created{type, id});
}


/** Called by the BlackBoardInterfaceObserver when an interface is destroyed
 * @param type Interface type name
 * @param id Interface ID
 */
void BlackboardListenerThread::bb_interface_destroyed(const char *type, const char *id) noexcept
{
  MutexLocker lock(&state_mutex_);
  iface_events_.emplace(new BlackboardListenerThread::Destroyed{type, id});
}


/** Called by the BlackBoardInterfaceListener when an interface changes
 * @param interface The changed interface
 */
void BlackboardListenerThread::bb_interface_data_changed(Interface *interface) noexcept
{
  MutexLocker lock(&state_mutex_);
  iface_events_.emplace(new BlackboardListenerThread::Changed{interface});
}


/** Test whether any events are in the queue
 * @return Whether any events are in the queue
 */
bool BlackboardListenerThread::event_pending()
{
  MutexLocker lock(&state_mutex_);
  return !iface_events_.empty();
}


/** Return and remove the next event in the queue
 * @return The next event
 */
shared_ptr<BlackboardListenerThread::Event> BlackboardListenerThread::event_pop()
{
  MutexLocker lock(&state_mutex_);
  shared_ptr<BlackboardListenerThread::Event> rv = iface_events_.front();
  iface_events_.pop();
  return rv;
}


/** Return an eclipse term representing the event
 * @return An eclipse term representing the event: bb_created(UID)
 */
BlackboardListenerThread::Created::operator EC_word ()
{ return ::term(EC_functor("bb_created", 1), uid().c_str()); }


/** Return an eclipse term representing the event
 * @return An eclipse term representing the event: bb_destroyed(UID)
 */
BlackboardListenerThread::Destroyed::operator EC_word ()
{ return ::term(EC_functor("bb_destroyed", 1), uid().c_str()); }


/** Return an eclipse term representing the event
 * @return An eclipse term representing the event: bb_changed(UID)
 */
BlackboardListenerThread::Changed::operator EC_word ()
{ return ::term(EC_functor("bb_changed", 1), uid().c_str()); }


