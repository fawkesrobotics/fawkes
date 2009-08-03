 
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
#include <core/threading/wait_condition.h>
#include <core/utils/lock_hashset.h>
#include <core/utils/lock_hashmap.h>
#include <utils/logging/liblogger.h>
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
  __bbil_writer_waitcond   = new WaitCondition(__bbil_writer_mutex);

  __bbil_reader_events     = 0;
  __bbil_reader_mutex      = new Mutex();
  __bbil_reader_waitcond   = new WaitCondition(__bbil_reader_mutex);

  __bbil_data_events       = 0;
  __bbil_data_mutex        = new Mutex();
  __bbil_data_waitcond     = new WaitCondition(__bbil_data_mutex);

  __bbil_messages_events   = 0;
  __bbil_messages_mutex    = new Mutex();
  __bbil_messages_waitcond = new WaitCondition(__bbil_messages_mutex);

  __bbio_events            = 0;
  __bbio_mutex             = new Mutex();
  __bbio_waitcond          = new WaitCondition(__bbio_mutex);
}


/** Destructor */
BlackBoardNotifier::~BlackBoardNotifier()
{
  delete __bbil_writer_waitcond;
  delete __bbil_writer_mutex;

  delete __bbil_reader_waitcond;
  delete __bbil_reader_mutex;

  delete __bbil_data_waitcond;
  delete __bbil_data_mutex;

  delete __bbil_messages_waitcond;
  delete __bbil_messages_mutex;

  delete __bbio_waitcond;
  delete __bbio_mutex;
}


/** Register BB event listener.
 * @param listener BlackBoard event listener to register
 * @param flags an or'ed combination of BBIL_FLAG_DATA, BBIL_FLAG_READER, BBIL_FLAG_WRITER
 * and BBIL_FLAG_INTERFACE. Only for the given types the event listener is registered.
 * BBIL_FLAG_ALL can be supplied to register for all events.
 */
void
BlackBoardNotifier::register_listener(BlackBoardInterfaceListener *listener,
				      unsigned int flags)
{
  if ( flags & BlackBoard::BBIL_FLAG_DATA ) {
    __bbil_data_mutex->lock();
    if (__bbil_data_events > 0) {
      LibLogger::log_warn("BlackBoardNotifier", "Registering interface listener %s "
			  "for data events queued",
			  listener->bbil_name());
      __bbil_data_queue.push_back(std::make_pair(true, listener));
    } else {
      add_listener(listener, listener->bbil_data_interfaces(), __bbil_data);
    }
    __bbil_data_mutex->unlock();
  }
  if ( flags & BlackBoard::BBIL_FLAG_MESSAGES ) {
    __bbil_messages_mutex->lock();
    BlackBoardInterfaceListener::InterfaceLockMapIterator i;
    BlackBoardInterfaceListener::InterfaceLockMap *im = listener->bbil_message_interfaces();
    MutexLocker lock(im->mutex());
    for (i = im->begin(); i != im->end(); ++i) {
      if ( ! i->second->is_writer() ||
	   (__bbil_messages.find(i->first) != __bbil_messages.end()) ) {
	__bbil_messages_mutex->unlock();
	throw Exception("An interface listener has already been registered for %s",
			i->first.c_str());
      }
    }
    for (i = im->begin(); i != im->end(); ++i) {
      __bbil_messages[i->first] = listener;
    }
    __bbil_messages_mutex->unlock();
  }
  if ( flags & BlackBoard::BBIL_FLAG_READER ) {
    __bbil_reader_mutex->lock();
    if (__bbil_reader_events > 0) {
      LibLogger::log_warn("BlackBoardNotifier", "Registering interface listener %s "
			  "for reader events queued",
			  listener->bbil_name());
      __bbil_reader_queue.push_back(std::make_pair(true, listener));
    } else {
      add_listener(listener, listener->bbil_reader_interfaces(), __bbil_reader);
    }
    __bbil_reader_mutex->unlock();
  }
  if ( flags & BlackBoard::BBIL_FLAG_WRITER ) {
    __bbil_writer_mutex->lock();
    if (__bbil_writer_events > 0) {
      LibLogger::log_warn("BlackBoardNotifier", "Registering interface listener %s "
			  "for writer events queued",
			  listener->bbil_name());
      __bbil_writer_queue.push_back(std::make_pair(true, listener));
    } else {
      add_listener(listener, listener->bbil_writer_interfaces(), __bbil_writer);
    }
    __bbil_writer_mutex->unlock();
  }
}


/** Unregister BB interface listener.
 * This will remove the given BlackBoard interface listener from any event that it was
 * previously registered for.
 * @param listener BlackBoard event listener to remove
 */
void
BlackBoardNotifier::unregister_listener(BlackBoardInterfaceListener *listener)
{
  remove_listener(listener, __bbil_writer_mutex, __bbil_writer_events,
		  __bbil_writer_queue, __bbil_writer);
  remove_listener(listener, __bbil_reader_mutex, __bbil_reader_events,
		  __bbil_reader_queue, __bbil_reader);
  remove_listener(listener, __bbil_data_mutex, __bbil_data_events,
		  __bbil_data_queue, __bbil_data);
  remove_message_listener(listener);
}

/** Add listener for specified map..
 * @param listener interface listener for events
 * @param im map of interfaces to listen for
 * @param ilmap internal map to add listener to
 */
void
BlackBoardNotifier::add_listener(BlackBoardInterfaceListener *listener,
				 BlackBoardInterfaceListener::InterfaceLockMap *im,
				 BBilMap &ilmap)
{
  BlackBoardInterfaceListener::InterfaceLockMapIterator i;
  im->lock();
  for (i = im->begin(); i != im->end(); ++i) {
    ilmap[i->first].push_back(listener);
  }
  im->unlock();
}

void
BlackBoardNotifier::remove_listener(BlackBoardInterfaceListener *listener,
				    Mutex *mutex, unsigned int events,
				    BBilQueue &queue, BBilMap &ilmap)
{
  MutexLocker lock(mutex);
  if (events > 0) {
    //LibLogger::log_warn("BlackBoardNotifier", "UN-registering interface listener %s queued",
    //			  listener->bbil_name());

    BBilQueue::iterator re;
    if ( (re = find(queue.begin(), queue.end(),
		    std::make_pair(true, listener))) != queue.end()) {
      // if there is an entry in the register queue, remove it!
      queue.erase(re);
    }
    queue.push_back(std::make_pair(false, listener));
  } else {
    remove_listener(ilmap, listener);
  }
}


/** Remove listener from map.
 * @param ilmap interface listener map to remove the listener from
 * @param listener listener to remove
 */
void
BlackBoardNotifier::remove_listener(BBilMap &ilmap, BlackBoardInterfaceListener *listener)
{
  BBilMapIterator i, tmp;

  i = ilmap.begin();;
  while (i != ilmap.end()) {
    BBilListIterator j = i->second.begin();
    while (j != i->second.end()) {
      if ( *j == listener ) {
	j = i->second.erase(j);
      } else {
	++j;
      }
    }
    if ( i->second.empty() ) {
      tmp = i;
      ++i;
      ilmap.erase(tmp);
    } else {
      ++i;
    }
  }
}


void
BlackBoardNotifier::remove_message_listener_map(BlackBoardInterfaceListener *listener)
{
  BBilMessageLockMapIterator i, tmp;

  i = __bbil_messages.begin();;
  while (i != __bbil_messages.end()) {
    if ( i->second == listener ) {
      // found!
      tmp = i;
      ++i;
      __bbil_messages.erase(tmp);
    } else {
      ++i;
    }
  }
}


void
BlackBoardNotifier::remove_message_listener(BlackBoardInterfaceListener *listener)
{
  __bbil_messages_mutex->lock();
  if (__bbil_messages_events > 0) {
    //LibLogger::log_warn("BlackBoardNotifier", "UN-registering interface (message) listener %s queued",
    //			listener->bbil_name());

    BBilQueue::iterator re;
    if ( (re = find(__bbil_messages_queue.begin(), __bbil_messages_queue.end(),
		    std::make_pair(true, listener))) != __bbil_messages_queue.end()) {
      // if there is an entry in the register queue, remove it!
      __bbil_messages_queue.erase(re);
    }
    __bbil_messages_queue.push_back(std::make_pair(false, listener));
  } else {
    remove_message_listener_map(listener);
  }
  __bbil_messages_mutex->unlock();
}


/** Register BB interface observer.
 * @param observer BlackBoard interface observer to register
 * @param flags an or'ed combination of BBIO_FLAG_CREATED, BBIO_FLAG_DESTROYED
 */
void
BlackBoardNotifier::register_observer(BlackBoardInterfaceObserver *observer,
				      unsigned int flags)
{
  __bbio_mutex->lock();
  if (__bbio_events > 0) {
    __bbio_queue.push_back(std::make_pair(flags, observer));
  } else {
    if ( flags & BlackBoard::BBIO_FLAG_CREATED ) {
      add_observer(observer, observer->bbio_get_observed_create(), __bbio_created);
    }
    if ( flags & BlackBoard::BBIO_FLAG_DESTROYED ) {
      add_observer(observer, observer->bbio_get_observed_destroy(), __bbio_destroyed);
    }
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
  if ( (lhmi = __bbio_created.find(type)) != __bbio_created.end() ) {
    BBioList &list = (*lhmi).second;
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
  if ( (lhmi = __bbio_destroyed.find(type)) != __bbio_destroyed.end() ) {
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
      __bbio_waitcond->wait();
    } else {
      while (! __bbio_queue.empty()) {
	BBioQueueEntry &e = __bbio_queue.front();
	if (e.first & BlackBoard::BBIO_FLAG_CREATED) { // register create
	  add_observer(e.second, e.second->bbio_get_observed_create(), __bbio_created);
	} else if (e.first & BlackBoard::BBIO_FLAG_DESTROYED) { // register destroy
	  add_observer(e.second, e.second->bbio_get_observed_destroy(), __bbio_destroyed);
	} else {       // unregister
	  remove_observer(__bbio_created, e.second);
	  remove_observer(__bbio_destroyed, e.second);
	}
	__bbio_queue.pop_front();
      }
      __bbio_waitcond->wake_all();
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
  if ( (__bbil_writer_events > 0) && ! __bbil_writer_queue.empty() ) {
    __bbil_writer_waitcond->wait();
  }
  __bbil_writer_events += 1;
  __bbil_writer_mutex->unlock();

  BBilMapIterator lhmi;
  BBilListIterator i, l;
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_writer.find(uid)) != __bbil_writer.end() ) {
    BBilList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
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
  if ( (__bbil_writer_events > 0) && ! __bbil_writer_queue.empty() ) {
    __bbil_writer_waitcond->wait();
  }
  __bbil_writer_events += 1;
  __bbil_writer_mutex->unlock();

  BBilMapIterator lhmi;
  BBilListIterator i, l;
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_writer.find(uid)) != __bbil_writer.end() ) {
    BBilList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
      Interface *bbil_iface = bbil->bbil_writer_interface(uid);
      if (bbil_iface != NULL) {
	if (bbil_iface->serial() != event_instance_serial) {
	  bbil->bb_interface_writer_removed(bbil_iface, event_instance_serial);
	}
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
      __bbil_writer_waitcond->wait();
    } else {
      while (! __bbil_writer_queue.empty()) {
	BBilQueueEntry &e = __bbil_writer_queue.front();
	if (e.first) { // register
	  add_listener(e.second, e.second->bbil_writer_interfaces(), __bbil_writer);
	} else {       // unregister
	  remove_listener(__bbil_writer, e.second);
	}
	__bbil_writer_queue.pop_front();
      }
      __bbil_writer_waitcond->wake_all();
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
  if ( (__bbil_reader_events > 0) && ! __bbil_reader_queue.empty() ) {
    __bbil_reader_waitcond->wait();
  }
  __bbil_reader_events += 1;
  __bbil_reader_mutex->unlock();

  BBilMapIterator lhmi;
  BBilListIterator i, l;
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_reader.find(uid)) != __bbil_reader.end() ) {
    BBilList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
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
  if ( (__bbil_reader_events > 0) && ! __bbil_reader_queue.empty() ) {
    __bbil_reader_waitcond->wait();
  }
  __bbil_reader_events += 1;
  __bbil_reader_mutex->unlock();

  BBilMapIterator lhmi;
  BBilListIterator i, l;
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_reader.find(uid)) != __bbil_reader.end() ) {
    BBilList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
      Interface *bbil_iface = bbil->bbil_reader_interface(uid);
      if (bbil_iface != NULL) {
	if (bbil_iface->serial() != event_instance_serial) {
	  bbil->bb_interface_reader_removed(bbil_iface, event_instance_serial);
	}
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
      __bbil_reader_waitcond->wait();
    } else {
      while (! __bbil_reader_queue.empty()) {
	BBilQueueEntry &e = __bbil_reader_queue.front();
	if (e.first) { // register
	  add_listener(e.second, e.second->bbil_reader_interfaces(), __bbil_reader);
	} else {       // unregister
	  remove_listener(__bbil_reader, e.second);
	}
	__bbil_reader_queue.pop_front();
      }
      __bbil_reader_waitcond->wake_all();
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
  if ( (__bbil_data_events > 0) && ! __bbil_data_queue.empty() ) {
    __bbil_data_waitcond->wait();
  }
  __bbil_data_events += 1;
  __bbil_data_mutex->unlock();

  BBilMapIterator lhmi;
  BBilListIterator i, l;
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_data.find(uid)) != __bbil_data.end() ) {
    BBilList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
      Interface *bbil_iface = bbil->bbil_data_interface(uid);
      if (bbil_iface != NULL) {
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
    if (__bbil_data_events > 0 ) {
      __bbil_data_waitcond->wait();
    } else {
      while (! __bbil_data_queue.empty()) {
	BBilQueueEntry &e = __bbil_data_queue.front();
	if (e.first) { // register
	  add_listener(e.second, e.second->bbil_data_interfaces(), __bbil_data);
	} else {       // unregister
	  remove_listener(__bbil_data, e.second);
	}
	__bbil_data_queue.pop_front();
      }
      __bbil_data_waitcond->wake_all();
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
 * @return true if any of the listeners did return true, false if none returned
 * true at all.
 * @see BlackBoardInterfaceListener::bb_interface_message_received()
 */
bool
BlackBoardNotifier::notify_of_message_received(const Interface *interface, Message *message)
{
  __bbil_messages_mutex->lock();
  if ( (__bbil_messages_events > 0) && ! __bbil_messages_queue.empty() ) {
    __bbil_messages_waitcond->wait();
  }
  __bbil_messages_events += 1;
  __bbil_messages_mutex->unlock();

  bool rv = false;

  const char *uid = interface->uid();
  if ( __bbil_messages.find(uid) != __bbil_messages.end() ) {
    BlackBoardInterfaceListener *bbil = __bbil_messages[uid];

    Interface *bbil_iface = bbil->bbil_message_interface(uid);
    if (bbil_iface != NULL ) {
      if ( bbil->bb_interface_message_received(bbil_iface, message) ) {
	  rv = true;
      }
    } else {
      LibLogger::log_warn("BlackBoardNotifier", "BBIL[%s] registered "
			  "for message received events for '%s' "
			  "but has no such interface",
			  bbil->bbil_name(), uid);
    }
  } else {
    rv = true;
  }

  __bbil_messages_mutex->lock();
  __bbil_messages_events -= 1;
  if ( ! __bbil_messages_queue.empty() ) {
    if (__bbil_messages_events > 0 ) {
      __bbil_messages_waitcond->wait();
    } else {
      while (! __bbil_messages_queue.empty()) {
	BBilQueueEntry &e = __bbil_messages_queue.front();
	// register never queues for message event listeners, only unregister does
	remove_message_listener_map(e.second);
	__bbil_messages_queue.pop_front();
      }
      __bbil_messages_waitcond->wake_all();
    }
  }
  __bbil_messages_mutex->unlock();

  return rv;
}

} // end namespace fawkes
