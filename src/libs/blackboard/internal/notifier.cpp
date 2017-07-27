 
/***************************************************************************
 *  notifier.cpp - BlackBoard notifier
 *
 *  Created: Mon Mar 03 23:28:18 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/internal/notifier.h>
#include <blackboard/blackboard.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/utils/lock_hashset.h>
#include <core/utils/lock_hashmap.h>
#include <logging/liblogger.h>
#include <interface/interface.h>

#include <algorithm>
#include <functional>
#include <cstdlib>
#include <cstring>
#include <fnmatch.h>

namespace fawkes {

/** @class BlackBoardNotifier <blackboard/internal/notifier.h>
 * BlackBoard notifier.
 * This class is used by the BlackBoard to notify listeners and observers
 * of changes. 
 *
 * @author Tim Niemueller
 */


/** Constructor. */
BlackBoardNotifier::BlackBoardNotifier()
{
  __bbil_writer_events     = 0;
  __bbil_writer_mutex      = new Mutex();

  __bbil_reader_events     = 0;
  __bbil_reader_mutex      = new Mutex();

  __bbil_data_events       = 0;
  __bbil_data_mutex        = new Mutex();

  __bbil_messages_events   = 0;
  __bbil_messages_mutex    = new Mutex();

  __bbio_events            = 0;
  __bbio_mutex             = new Mutex();
}


/** Destructor */
BlackBoardNotifier::~BlackBoardNotifier()
{
  delete __bbil_writer_mutex;
  delete __bbil_reader_mutex;
  delete __bbil_data_mutex;
  delete __bbil_messages_mutex;

  delete __bbio_mutex;
}

/** Register BB event listener.
 * @param listener BlackBoard event listener to register
 * @param flag concatenation of flags denoting which queue entries should be
 * processed
 */
void
BlackBoardNotifier::register_listener(BlackBoardInterfaceListener *listener,
                                      BlackBoard::ListenerRegisterFlag flag)
{
  update_listener(listener, flag);
}


/** Update BB event listener.
 * @param listener BlackBoard event listener to update subscriptions of
 * @param flag concatenation of flags denoting which queue entries should be
 * processed
 */
void
BlackBoardNotifier::update_listener(BlackBoardInterfaceListener *listener,
                                    BlackBoard::ListenerRegisterFlag flag)
{
  const BlackBoardInterfaceListener::InterfaceQueue & queue =
    listener->bbil_acquire_queue();

  BlackBoardInterfaceListener::InterfaceQueue::const_iterator i = queue.begin();

  for (i = queue.begin(); i != queue.end(); ++i) {
    switch (i->type) {
    case BlackBoardInterfaceListener::DATA:
      if (flag & BlackBoard::BBIL_FLAG_DATA) {
        proc_listener_maybe_queue(i->op, i->interface, listener,
                                  __bbil_data_mutex, __bbil_data_events,
                                  __bbil_data, __bbil_data_queue, "data");
      }
      break;
    case BlackBoardInterfaceListener::MESSAGES:
      if (flag & BlackBoard::BBIL_FLAG_MESSAGES) {
        proc_listener_maybe_queue(i->op, i->interface, listener,
                                  __bbil_messages_mutex, __bbil_messages_events,
                                  __bbil_messages, __bbil_messages_queue,
                                  "messages");
      }
      break;
    case BlackBoardInterfaceListener::READER:
      if (flag & BlackBoard::BBIL_FLAG_READER) {
      proc_listener_maybe_queue(i->op, i->interface, listener,
                                __bbil_reader_mutex, __bbil_reader_events,
                                __bbil_reader, __bbil_reader_queue, "reader");
      }
      break;
    case BlackBoardInterfaceListener::WRITER:
      if (flag & BlackBoard::BBIL_FLAG_WRITER) {
      proc_listener_maybe_queue(i->op, i->interface, listener,
                                __bbil_writer_mutex, __bbil_writer_events,
                                __bbil_writer, __bbil_writer_queue, "writer");
      }
      break;
    default: break;
    }
  }

  listener->bbil_release_queue(flag);
}

void
BlackBoardNotifier::proc_listener_maybe_queue(bool op,
                                              Interface *interface,
                                              BlackBoardInterfaceListener *listener,
                                              Mutex *mutex, unsigned int &events,
                                              BBilMap &map, BBilQueue &queue,
                                              const char *hint)
{
  MutexLocker lock(mutex);
  if (events > 0) {
    LibLogger::log_warn("BlackBoardNotifier", "%s interface "
                        "listener %s for %s events (queued)",
                        op ? "Registering" : "Unregistering",
                        listener->bbil_name(), hint);

    queue_listener(op, interface, listener, queue);
  } else {
    if (op) { // add
      add_listener(interface, listener, map);
    } else {
      remove_listener(interface, listener, map);
    }
  }
}


/** Unregister BB interface listener.
 * This will remove the given BlackBoard interface listener from any
 * event that it was previously registered for.
 * @param listener BlackBoard event listener to remove
 */
void
BlackBoardNotifier::unregister_listener(BlackBoardInterfaceListener *listener)
{
  const BlackBoardInterfaceListener::InterfaceMaps maps =
    listener->bbil_acquire_maps();

  BlackBoardInterfaceListener::InterfaceMap::const_iterator i;
  for (i = maps.data.begin(); i != maps.data.end(); ++i) {
    proc_listener_maybe_queue(false, i->second, listener,
                              __bbil_data_mutex, __bbil_data_events,
                              __bbil_data, __bbil_data_queue, "data");
  }

  for (i = maps.messages.begin(); i != maps.messages.end(); ++i) {
    proc_listener_maybe_queue(false, i->second, listener,
                              __bbil_messages_mutex, __bbil_messages_events,
                              __bbil_messages, __bbil_messages_queue,
                              "messages");
  }

  for (i = maps.reader.begin(); i != maps.reader.end(); ++i) {
    proc_listener_maybe_queue(false, i->second, listener,
                              __bbil_reader_mutex, __bbil_reader_events,
                              __bbil_reader, __bbil_reader_queue, "reader");
  }

  for (i = maps.writer.begin(); i != maps.writer.end(); ++i) {
    proc_listener_maybe_queue(false, i->second, listener,
                              __bbil_writer_mutex, __bbil_writer_events,
                              __bbil_writer, __bbil_writer_queue, "writer");
  }

  listener->bbil_release_maps();
}

/** Add listener for specified map.
 * @param listener interface listener for events
 * @param im map of interfaces to listen for
 * @param ilmap internal map to add listener to
 */
void
BlackBoardNotifier::add_listener(Interface *interface,
                                 BlackBoardInterfaceListener *listener,
                                 BBilMap &ilmap)
{
  std::pair<BBilMap::iterator, BBilMap::iterator> ret =
    ilmap.equal_range(interface->uid());

  BBilMap::value_type v = std::make_pair(interface->uid(), listener);
  BBilMap::iterator f = std::find(ret.first, ret.second, v);

  if (f == ret.second) {
    ilmap.insert(std::make_pair(interface->uid(), listener));
  }
}

void
BlackBoardNotifier::remove_listener(Interface *interface,
                                    BlackBoardInterfaceListener *listener,
                                    BBilMap &ilmap)
{
  std::pair<BBilMap::iterator, BBilMap::iterator> ret =
    ilmap.equal_range(interface->uid());
  for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
    if (j->second == listener) {
      ilmap.erase(j);
      break;
    }
  }
}


bool
BlackBoardNotifier::is_in_queue(bool op, BBilQueue &queue, const char *uid,
                                BlackBoardInterfaceListener *bbil)
{
  BBilQueue::iterator q;
  for (q = queue.begin(); q != queue.end(); ++q) {
    if ((q->op == op) && (q->uid == uid) && (q->listener == bbil)) {
      return true;
    }
  }
  return false;
}

void
BlackBoardNotifier::queue_listener(bool op, Interface *interface,
                                   BlackBoardInterfaceListener *listener,
                                   BBilQueue &queue)
{
  BBilQueueEntry qe = { op, interface->uid(), interface, listener };
  queue.push_back(qe);
}



/** Register BB interface observer.
 * @param observer BlackBoard interface observer to register
 */
void
BlackBoardNotifier::register_observer(BlackBoardInterfaceObserver *observer)
{
  __bbio_mutex->lock();
  if (__bbio_events > 0) {
    __bbio_queue.push_back(std::make_pair(1, observer));
  } else {
    add_observer(observer, observer->bbio_get_observed_create(), __bbio_created);
    add_observer(observer, observer->bbio_get_observed_destroy(), __bbio_destroyed);
  }
  __bbio_mutex->unlock();
}


void
BlackBoardNotifier::add_observer(BlackBoardInterfaceObserver *observer,
				 BlackBoardInterfaceObserver::ObservedInterfaceLockMap *its,
				 BBioMap &bbiomap)
{
  BlackBoardInterfaceObserver::ObservedInterfaceLockMapIterator i;
  its->lock();
  for (i = its->begin(); i != its->end(); ++i) {
    bbiomap[i->first].push_back(make_pair(observer, i->second));
  }
  its->unlock();
}


/** Remove observer from map.
 * @param iomap interface observer map to remove the observer from
 * @param observer observer to remove
 */
void
BlackBoardNotifier::remove_observer(BBioMap &iomap, BlackBoardInterfaceObserver *observer)
{
  BBioMapIterator i, tmp;

  i = iomap.begin();
  while (i != iomap.end()) {
    BBioListIterator j = i->second.begin();
    while (j != i->second.end()) {
      if ( j->first == observer ) {
	j = i->second.erase(j);
      } else {
	++j;
      }
    }
    if ( i->second.empty() ) {
      tmp = i;
      ++i;
      iomap.erase(tmp);
    } else {
      ++i;
    }
  }
}

/** Unregister BB interface observer.
 * This will remove the given BlackBoard event listener from any event that it was
 * previously registered for.
 * @param observer BlackBoard event listener to remove
 */
void
BlackBoardNotifier::unregister_observer(BlackBoardInterfaceObserver *observer)
{
  MutexLocker lock(__bbio_mutex);
  if ( __bbio_events > 0) {
    BBioQueueEntry e = std::make_pair((unsigned int)0, observer);
    BBioQueue::iterator re;
    while ( (re = find_if(__bbio_queue.begin(), __bbio_queue.end(),
			  bind2nd(std::not_equal_to<BBioQueueEntry>(), e)))
	    != __bbio_queue.end()) {
      // if there is an entry in the register queue, remove it!
      if (re->second == observer) {
	__bbio_queue.erase(re);
      }
    }
    __bbio_queue.push_back(std::make_pair(0, observer));

  } else {
    remove_observer(__bbio_created, observer);
    remove_observer(__bbio_destroyed, observer);
  }
}

/** Notify that an interface has been created.
 * @param type type of the interface
 * @param id ID of the interface
 */
void
BlackBoardNotifier::notify_of_interface_created(const char *type, const char *id) throw()
{
  __bbio_mutex->lock();
  __bbio_events += 1;
  __bbio_mutex->unlock();

  BBioMapIterator lhmi;
  BBioListIterator i, l;
  for (lhmi = __bbio_created.begin(); lhmi != __bbio_created.end(); ++lhmi) {
    if (fnmatch(lhmi->first.c_str(), type, 0) != 0) continue;

    BBioList &list = lhmi->second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceObserver *bbio = i->first;
      for (std::list<std::string>::iterator pi = i->second.begin(); pi != i->second.end(); ++pi) {
	if (fnmatch(pi->c_str(), id, 0) == 0) {
	  bbio->bb_interface_created(type, id);
	  break;
	}
      }
    }
  }

  __bbio_mutex->lock();
  __bbio_events -= 1;
  process_bbio_queue();
  __bbio_mutex->unlock();
}


/** Notify that an interface has been destroyed.
 * @param type type of the interface
 * @param id ID of the interface
 */
void
BlackBoardNotifier::notify_of_interface_destroyed(const char *type, const char *id) throw()
{
  __bbio_mutex->lock();
  __bbio_events += 1;
  __bbio_mutex->unlock();

  BBioMapIterator lhmi;
  BBioListIterator i, l;
  for (lhmi = __bbio_destroyed.begin(); lhmi != __bbio_destroyed.end(); ++lhmi) {
    if (fnmatch(lhmi->first.c_str(), type, 0) != 0) continue;

    BBioList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceObserver *bbio = i->first;
      for (std::list<std::string>::iterator pi = i->second.begin(); pi != i->second.end(); ++pi) {
	if (fnmatch(pi->c_str(), id, 0) == 0) {
	  bbio->bb_interface_destroyed(type, id);
	  break;
	}
      }
    }
  }

  __bbio_mutex->lock();
  __bbio_events -= 1;
  process_bbio_queue();
  __bbio_mutex->unlock();
}


void
BlackBoardNotifier::process_bbio_queue()
{
  if ( ! __bbio_queue.empty() ) {
    if (__bbio_events > 0 ) {
      return;
    } else {
      while (! __bbio_queue.empty()) {
	BBioQueueEntry &e = __bbio_queue.front();
	if (e.first) { // register
	  add_observer(e.second, e.second->bbio_get_observed_create(), __bbio_created);
	  add_observer(e.second, e.second->bbio_get_observed_destroy(), __bbio_destroyed);
	} else {       // unregister
	  remove_observer(__bbio_created, e.second);
	  remove_observer(__bbio_destroyed, e.second);
	}
	__bbio_queue.pop_front();
      }
    }
  }
}


/** Notify that writer has been added.
 * @param interface the interface for which the event happened. It is not necessarily the
 * instance which caused the event, but it must have the same mem serial.
 * @param event_instance_serial the instance serial of the interface that caused the event
 * @see BlackBoardInterfaceListener::bb_interface_writer_added()
 */
void
BlackBoardNotifier::notify_of_writer_added(const Interface *interface,
					   unsigned int event_instance_serial) throw()
{
  __bbil_writer_mutex->lock();
  __bbil_writer_events += 1;
  __bbil_writer_mutex->unlock();

  const char *uid = interface->uid();
  std::pair<BBilMap::iterator, BBilMap::iterator> ret =
    __bbil_writer.equal_range(uid);
  for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
    BlackBoardInterfaceListener *bbil = j->second;
    if (! is_in_queue(/* remove op*/ false, __bbil_writer_queue, uid, bbil)) {
      Interface *bbil_iface = bbil->bbil_writer_interface(uid);
      if (bbil_iface != NULL ) {
        bbil->bb_interface_writer_added(bbil_iface, event_instance_serial);
      } else {
        LibLogger::log_warn("BlackBoardNotifier",
                            "BBIL[%s] registered for writer events "
                            "(open) for '%s' but has no such interface",
                            bbil->bbil_name(), uid);
      }
    }
  }

  __bbil_writer_mutex->lock();
  __bbil_writer_events -= 1;
  process_writer_queue();
  __bbil_writer_mutex->unlock();
}


/** Notify that writer has been removed.
 * @param interface interface for which the writer has been removed
 * @param event_instance_serial instance serial of the interface that caused the event
 * @see BlackBoardInterfaceListener::bb_interface_writer_removed()
 */
void
BlackBoardNotifier::notify_of_writer_removed(const Interface *interface,
					     unsigned int event_instance_serial) throw()
{
  __bbil_writer_mutex->lock();
  __bbil_writer_events += 1;
  __bbil_writer_mutex->unlock();

  const char *uid = interface->uid();
  std::pair<BBilMap::iterator, BBilMap::iterator> ret =
    __bbil_writer.equal_range(uid);
  for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
    BlackBoardInterfaceListener *bbil = j->second;
    if (! is_in_queue(/* remove op*/ false, __bbil_data_queue, uid, bbil)) {
      Interface *bbil_iface = bbil->bbil_writer_interface(uid);
      if (bbil_iface != NULL ) {
        bbil->bb_interface_writer_removed(bbil_iface, event_instance_serial);
      } else {
        LibLogger::log_warn("BlackBoardNotifier",
                            "BBIL[%s] registered for writer events "
                            "(close) for '%s' but has no such interface",
                            bbil->bbil_name(), uid);
      }
    }
  }

  __bbil_writer_mutex->lock();
  __bbil_writer_events -= 1;
  process_writer_queue();
  __bbil_writer_mutex->unlock();
}

void
BlackBoardNotifier::process_writer_queue()
{
  if ( ! __bbil_writer_queue.empty() ) {
    if (__bbil_writer_events > 0 ) {
      return;
    } else {
      while (! __bbil_writer_queue.empty()) {
	BBilQueueEntry &e = __bbil_writer_queue.front();
	if (e.op) { // register
	  add_listener(e.interface, e.listener, __bbil_writer);
	} else {    // unregister
	  remove_listener(e.interface, e.listener, __bbil_writer);
	}
	__bbil_writer_queue.pop_front();
      }
    }
  }
}


/** Notify that reader has been added.
 * @param interface interface for which the reader has been added
 * @param event_instance_serial instance serial of the interface that caused the event
 * @see BlackBoardInterfaceListener::bb_interface_reader_added()
 */
void
BlackBoardNotifier::notify_of_reader_added(const Interface *interface,
					   unsigned int event_instance_serial) throw()
{
  __bbil_reader_mutex->lock();
  __bbil_reader_events += 1;
  __bbil_reader_mutex->unlock();

  const char *uid = interface->uid();
  std::pair<BBilMap::iterator, BBilMap::iterator> ret =
    __bbil_reader.equal_range(uid);
  for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
    BlackBoardInterfaceListener *bbil = j->second;
    if (! is_in_queue(/* remove op*/ false, __bbil_reader_queue, uid, bbil)) {
      Interface *bbil_iface = bbil->bbil_reader_interface(uid);
      if (bbil_iface != NULL ) {
        bbil->bb_interface_reader_added(bbil_iface, event_instance_serial);
      } else {
        LibLogger::log_warn("BlackBoardNotifier",
                            "BBIL[%s] registered for reader events "
                            "(open) for '%s' but has no such interface",
                            bbil->bbil_name(), uid);
      }
    }
  }

  __bbil_reader_mutex->lock();
  __bbil_reader_events -= 1;
  process_reader_queue();
  __bbil_reader_mutex->unlock();
}


/** Notify that reader has been removed.
 * @param interface interface for which the reader has been removed
 * @param event_instance_serial instance serial of the interface that caused the event
 * @see BlackBoardInterfaceListener::bb_interface_reader_removed()
 */
void
BlackBoardNotifier::notify_of_reader_removed(const Interface *interface,
					     unsigned int event_instance_serial) throw()
{
  __bbil_reader_mutex->lock();
  __bbil_reader_events += 1;
  __bbil_reader_mutex->unlock();

  const char *uid = interface->uid();
  std::pair<BBilMap::iterator, BBilMap::iterator> ret =
    __bbil_reader.equal_range(uid);
  for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
    BlackBoardInterfaceListener *bbil = j->second;
    if (! is_in_queue(/* remove op*/ false, __bbil_data_queue, uid, bbil)) {
      Interface *bbil_iface = bbil->bbil_reader_interface(uid);
      if (bbil_iface != NULL ) {
        bbil->bb_interface_reader_removed(bbil_iface, event_instance_serial);
      } else {
        LibLogger::log_warn("BlackBoardNotifier",
                            "BBIL[%s] registered for reader events "
                            "(close) for '%s' but has no such interface",
                            bbil->bbil_name(), uid);
      }
    }
  }

  __bbil_reader_mutex->lock();
  __bbil_reader_events -= 1;
  process_reader_queue();
  __bbil_reader_mutex->unlock();
}


void
BlackBoardNotifier::process_reader_queue()
{
  if ( ! __bbil_reader_queue.empty() ) {
    if (__bbil_reader_events > 0 ) {
      return;
    } else {
      while (! __bbil_reader_queue.empty()) {
	BBilQueueEntry &e = __bbil_reader_queue.front();
	if (e.op) { // register
	  add_listener(e.interface, e.listener, __bbil_reader);
	} else {    // unregister
	  remove_listener(e.interface, e.listener, __bbil_reader);
	}
	__bbil_reader_queue.pop_front();
      }
    }
  }
}


/** Notify of data change.
 * Notify all subscribers of the given interface of a data change.
 * This also influences logging and sending data over the network so it is
 * mandatory to call this function! The interface base class write method does
 * that for you.
 * @param interface interface whose subscribers to notify
 * @see Interface::write()
 * @see BlackBoardInterfaceListener::bb_interface_data_changed()
 */
void
BlackBoardNotifier::notify_of_data_change(const Interface *interface)
{
  __bbil_data_mutex->lock();
  __bbil_data_events += 1;
  __bbil_data_mutex->unlock();

  const char *uid = interface->uid();
  std::pair<BBilMap::iterator, BBilMap::iterator> ret =
    __bbil_data.equal_range(uid);
  for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
    BlackBoardInterfaceListener *bbil = j->second;
    if (! is_in_queue(/* remove op*/ false, __bbil_data_queue, uid, bbil)) {
      Interface *bbil_iface = bbil->bbil_data_interface(uid);
      if (bbil_iface != NULL ) {
        bbil->bb_interface_data_changed(bbil_iface);
      } else {
        LibLogger::log_warn("BlackBoardNotifier",
                            "BBIL[%s] registered for data change events "
                            "for '%s' but has no such interface",
                            bbil->bbil_name(), uid);
      }
    }
  }

  __bbil_data_mutex->lock();
  __bbil_data_events -= 1;
  if ( ! __bbil_data_queue.empty() ) {
    if (__bbil_data_events == 0 ) {
      while (! __bbil_data_queue.empty()) {
	BBilQueueEntry &e = __bbil_data_queue.front();
	if (e.op) { // register
	  add_listener(e.interface, e.listener,  __bbil_data);
	} else {    // unregister
	  remove_listener(e.interface, e.listener,  __bbil_data);
	}
	__bbil_data_queue.pop_front();
      }
    }
  }
  __bbil_data_mutex->unlock();
}


/** Notify of message received
 * Notify all subscribers of the given interface of an incoming message
 * This also influences logging and sending data over the network so it is
 * mandatory to call this function! The interface base class write method does
 * that for you.
 * @param interface interface whose subscribers to notify
 * @param message message which is being received
 * @return false if any listener returned false, true otherwise
 * @see BlackBoardInterfaceListener::bb_interface_message_received()
 */
bool
BlackBoardNotifier::notify_of_message_received(const Interface *interface, Message *message)
{
  __bbil_messages_mutex->lock();
  __bbil_messages_events += 1;
  __bbil_messages_mutex->unlock();

  bool enqueue = true;

  const char *uid = interface->uid();
  std::pair<BBilMap::iterator, BBilMap::iterator> ret =
    __bbil_messages.equal_range(uid);
  for (BBilMap::iterator j = ret.first; j != ret.second; ++j) {
    BlackBoardInterfaceListener *bbil = j->second;
    if (! is_in_queue(/* remove op*/ false, __bbil_messages_queue, uid, bbil)) {
      Interface *bbil_iface = bbil->bbil_message_interface(uid);
      if (bbil_iface != NULL ) {
        bool abort = ! bbil->bb_interface_message_received(bbil_iface, message);
        if (abort) {
          enqueue = false;
          break;
        }
      } else {
        LibLogger::log_warn("BlackBoardNotifier",
                          "BBIL[%s] registered for message events "
                            "for '%s' but has no such interface",
                            bbil->bbil_name(), uid);
      }
    }
  }

  __bbil_messages_mutex->lock();
  __bbil_messages_events -= 1;
  if ( ! __bbil_messages_queue.empty() ) {
    if (__bbil_messages_events == 0 ) {
      while (! __bbil_messages_queue.empty()) {
	BBilQueueEntry &e = __bbil_messages_queue.front();
	if (e.op) { // register
	  add_listener(e.interface, e.listener,  __bbil_messages);
	} else {    // unregister
	  remove_listener(e.interface, e.listener,  __bbil_messages);
	}
	__bbil_messages_queue.pop_front();
      }
    }
  }
  __bbil_messages_mutex->unlock();

  return enqueue;
}

} // end namespace fawkes
