 
/***************************************************************************
 *  notifier.cpp - BlackBoard notifier
 *
 *  Created: Mon Mar 03 23:28:18 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <blackboard/notifier.h>
#include <blackboard/blackboard.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>

#include <core/utils/lock_hashset.h>
#include <core/utils/lock_hashmap.h>
#include <utils/logging/liblogger.h>
#include <interface/interface.h>

#include <cstdlib>
#include <cstring>

/** @class BlackBoardNotifier <blackboard/notifier.h>
 * BlackBoard notifier.
 * This class is used by the BlackBoard to notify listeners and observers
 * of changes. 
 *
 * @author Tim Niemueller
 */


/** Constructor. */
BlackBoardNotifier::BlackBoardNotifier()
{
}


/** Destructor */
BlackBoardNotifier::~BlackBoardNotifier()
{
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
    BlackBoardInterfaceListener::InterfaceLockHashMapIterator i;
    BlackBoardInterfaceListener::InterfaceLockHashMap *im = listener->bbil_data_interfaces();
    __bbil_data.lock();
    for (i = im->begin(); i != im->end(); ++i) {
      __bbil_data[i->first].push_back(listener);
    }
    __bbil_data.unlock();
  }
  if ( flags & BlackBoard::BBIL_FLAG_MESSAGES ) {
    BlackBoardInterfaceListener::InterfaceLockHashMapIterator i;
    BlackBoardInterfaceListener::InterfaceLockHashMap *im = listener->bbil_message_interfaces();
    __bbil_messages.lock();
    for (i = im->begin(); i != im->end(); ++i) {
      if ( i->second->is_writer() ) {
	__bbil_messages[i->first].push_back(listener);
      }
    }
    __bbil_messages.unlock();
  }
  if ( flags & BlackBoard::BBIL_FLAG_READER ) {
    BlackBoardInterfaceListener::InterfaceLockHashMapIterator i;
    BlackBoardInterfaceListener::InterfaceLockHashMap *im = listener->bbil_reader_interfaces();
    __bbil_reader.lock();
    for (i = im->begin(); i != im->end(); ++i) {
      __bbil_reader[i->first].push_back(listener);
    }
    __bbil_reader.unlock();
  }
  if ( flags & BlackBoard::BBIL_FLAG_WRITER ) {
    BlackBoardInterfaceListener::InterfaceLockHashMapIterator i;
    BlackBoardInterfaceListener::InterfaceLockHashMap *im = listener->bbil_writer_interfaces();
    __bbil_writer.lock();
    for (i = im->begin(); i != im->end(); ++i) {
      __bbil_writer[i->first].push_back(listener);
    }
    __bbil_writer.unlock();
  }
}


/** Remove listener from map.
 * @param ilmap interface listener map to remove the listener from
 * @param listener listener to remove
 */
void
BlackBoardNotifier::remove_listener(BBilLockMap &ilmap, BlackBoardInterfaceListener *listener)
{
  BBilLockMapIterator i, tmp;

  ilmap.lock();
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
  ilmap.unlock();
}

/** Unregister BB interface listener.
 * This will remove the given BlackBoard interface listener from any event that it was
 * previously registered for.
 * @param listener BlackBoard event listener to remove
 */
void
BlackBoardNotifier::unregister_listener(BlackBoardInterfaceListener *listener)
{
  remove_listener(__bbil_data, listener);
  remove_listener(__bbil_messages, listener);
  remove_listener(__bbil_reader, listener);
  remove_listener(__bbil_writer, listener);
}


/** Register BB interface observer.
 * @param observer BlackBoard interface observer to register
 * @param flags an or'ed combination of BBIO_FLAG_CREATED, BBIO_FLAG_DESTROYED
 */
void
BlackBoardNotifier::register_observer(BlackBoardInterfaceObserver *observer,
					      unsigned int flags)
{
  if ( flags & BlackBoard::BBIO_FLAG_CREATED ) {
    BlackBoardInterfaceObserver::InterfaceTypeLockHashSetIterator i;
    BlackBoardInterfaceObserver::InterfaceTypeLockHashSet *its = observer->bbio_interface_create_types();
    __bbio_created.lock();
    for (i = its->begin(); i != its->end(); ++i) {
      __bbio_created[*i].push_back(observer);
    }
    __bbio_created.unlock();
  }

  if ( flags & BlackBoard::BBIO_FLAG_DESTROYED ) {
    BlackBoardInterfaceObserver::InterfaceTypeLockHashSetIterator i;
    BlackBoardInterfaceObserver::InterfaceTypeLockHashSet *its = observer->bbio_interface_destroy_types();
    __bbio_destroyed.lock();
    for (i = its->begin(); i != its->end(); ++i) {
      __bbio_destroyed[*i].push_back(observer);
    }
    __bbio_destroyed.unlock();
  }
}


/** Remove observer from map.
 * @param iomap interface observer map to remove the observer from
 * @param observer observer to remove
 */
void
BlackBoardNotifier::remove_observer(BBioLockMap &iomap, BlackBoardInterfaceObserver *observer)
{
  BBioLockMapIterator i, tmp;

  iomap.lock();
  i = iomap.begin();;
  while (i != iomap.end()) {
    BBioListIterator j = i->second.begin();
    while (j != i->second.end()) {
      if ( *j == observer ) {
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
  iomap.unlock();
}

/** Unregister BB interface observer.
 * This will remove the given BlackBoard event listener from any event that it was
 * previously registered for.
 * @param observer BlackBoard event listener to remove
 */
void
BlackBoardNotifier::unregister_observer(BlackBoardInterfaceObserver *observer)
{
  remove_observer(__bbio_created, observer);
  remove_observer(__bbio_destroyed, observer);
}

/** Notify that an interface has been created.
 * @param type type of the interface
 * @param id ID of the interface
 */
void
BlackBoardNotifier::notify_of_interface_created(const char *type, const char *id) throw()
{
  BBioLockMapIterator lhmi;
  BBioListIterator i, l;
  __bbio_created.lock();
  if ( (lhmi = __bbio_created.find(type)) != __bbio_created.end() ) {
    BBioList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceObserver *bbio = (*i);
      bbio->bb_interface_created(type, id);
    }
  }
  __bbio_created.unlock();
}


/** Notify that an interface has been destroyed.
 * @param type type of the interface
 * @param id ID of the interface
 */
void
BlackBoardNotifier::notify_of_interface_destroyed(const char *type, const char *id) throw()
{
  BBioLockMapIterator lhmi;
  BBioListIterator i, l;
  __bbio_destroyed.lock();
  if ( (lhmi = __bbio_destroyed.find(type)) != __bbio_destroyed.end() ) {
    BBioList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceObserver *bbio = (*i);
      bbio->bb_interface_destroyed(type, id);
    }
  }
  __bbio_destroyed.unlock();
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
  BBilLockMapIterator lhmi;
  BBilListIterator i, l;
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_writer.find(uid)) != __bbil_writer.end() ) {
    BBilList &list = (*lhmi).second;
    __bbil_writer.lock();
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
      Interface *bbil_iface = bbil->bbil_writer_interface(uid);
      if (bbil_iface != NULL ) {
	bbil->bb_interface_writer_added(bbil_iface, event_instance_serial);
      } else {
	LibLogger::log_warn("BlackBoardNotifier", "BBIL registered for writer "
			    "events (open) for '%s' but has no such interface", uid);
      }
    }
    __bbil_writer.unlock();
  }
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
  BBilLockMapIterator lhmi;
  BBilListIterator i, l;
  __bbil_writer.lock();
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
	LibLogger::log_warn("BlackBoardNotifier", "BBIL registered for writer "
			    "events (close) for '%s' but has no such interface", uid);
      }
    }
  }
  __bbil_writer.unlock();
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
  BBilLockMapIterator lhmi;
  BBilListIterator i, l;
  __bbil_reader.lock();
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_reader.find(uid)) != __bbil_reader.end() ) {
    BBilList &list = (*lhmi).second;
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
      Interface *bbil_iface = bbil->bbil_reader_interface(uid);
      if (bbil_iface != NULL ) {
	bbil->bb_interface_reader_added(bbil_iface, event_instance_serial);
      } else {
	LibLogger::log_warn("BlackBoardNotifier", "BBIL registered for reader "
			    "events (open) for '%s' but has no such interface", uid);
      }
    }
  }
  __bbil_reader.unlock();
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
  BBilLockMapIterator lhmi;
  BBilListIterator i, l;
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_reader.find(uid)) != __bbil_reader.end() ) {
    BBilList &list = (*lhmi).second;
    __bbil_reader.lock();
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
      Interface *bbil_iface = bbil->bbil_reader_interface(uid);
      if (bbil_iface != NULL) {
	if (bbil_iface->serial() != event_instance_serial) {
	  bbil->bb_interface_reader_removed(bbil_iface, event_instance_serial);
	}
      } else {
	LibLogger::log_warn("BlackBoardNotifier", "BBIL registered for reader "
			    "events (close) for '%s' but has no such interface", uid);
      }
    }
    __bbil_reader.unlock();
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
  BBilLockMapIterator lhmi;
  BBilListIterator i, l;
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_data.find(uid)) != __bbil_data.end() ) {
    BBilList &list = (*lhmi).second;
    __bbil_data.lock();
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
      Interface *bbil_iface = bbil->bbil_data_interface(uid);
      if (bbil_iface != NULL) {
	if (bbil_iface->serial() != interface->serial()) {
	  bbil->bb_interface_data_changed(bbil_iface);
	}
      } else {
	LibLogger::log_warn("BlackBoardNotifier", "BBIL registered for data change "
			    "events for '%s' but has no such interface", uid);
      }
    }
    __bbil_data.unlock();
  }
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
  BBilLockMapIterator lhmi;
  BBilListIterator i, l;
  bool rv = __bbil_messages.empty();
  const char *uid = interface->uid();
  if ( (lhmi = __bbil_messages.find(uid)) != __bbil_messages.end() ) {
    BBilList &list = (*lhmi).second;
    __bbil_messages.lock();
    for (i = list.begin(); i != list.end(); ++i) {
      BlackBoardInterfaceListener *bbil = (*i);
      Interface *bbil_iface = bbil->bbil_message_interface(uid);
      if (bbil_iface != NULL ) {
	if ( bbil->bb_interface_message_received(bbil_iface, message) ) {
	  rv = true;
	}
      } else {
	LibLogger::log_warn("BlackBoardNotifier", "BBIL registered for message received "
			    "events for '%s' but has no such interface", uid);
      }
    }
    __bbil_messages.unlock();
  }

  return rv;
}
