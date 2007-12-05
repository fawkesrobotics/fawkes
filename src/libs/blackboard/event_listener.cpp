 
/***************************************************************************
 *  event_listener.cpp - BlackBoard event listener
 *
 *  Created: Wed Nov 08 10:00:34 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <blackboard/event_listener.h>
#include <interface/interface.h>

/** @class BlackBoardEventListener <blackboard/event_listener.h>
 * BlackBoard event listener.
 * Derive this class if you want to be notified of specific BlackBoard 
 * events.
 *
 * The bb_* methods are called during the appropriate operation. The operation
 * that you carry out in this event handler really has to damn fast, or the
 * performance of the whole system will suffer severely. For this reason use
 * this notification facility only rarely and only register for the appropriate
 * events.
 *
 * This class provides the basic infrastructure that can be used to build
 * your own event handler. During the life time of your event handler your
 * first add all the interfaces to the appropriate structures that you want
 * to listen for and add the interface types where you want to be notified
 * of creation events.
 *
 * The reader/writer added/removed and data changed notifications act upon a
 * specific interface. Any modification done with any instance of the interface
 * is reported to you. The interface creation notification deals only
 * with types of interfaces. There is no interface deletion notification because
 * the general idea is that you opened the interface by yourself for reading and
 * thus the deletion will not happen before you close the interface.
 *
 * Here is a simple life cycle of a BlackBoard event listener:
 * First you create your interface that you want to listen for.
 * The protected methods bbel_add_data_interface(), bbel_add_reader_interface(),
 * bbel_add_writer_interface() and bbel_add_interface_create_type() have to
 * be called with the appropriate interfaces <i>before</i> the event handler is
 * actually registered with the interface manager! Also add any interface type
 * that you want to get interface creation/deletion notifications for.
 * After this you register your event handler with the interface manager. From
 * now on will be called for the all registered events.
 * In the end you unregister the event listener and <i>then</i> close any
 * interface that you had registered before.
 *
 * It is important that you first unregister as an event handler before closing
 * the interface. Otherwise it could happen that you close the interface and
 * the instance is deleted and afterwards an event for that very interface
 * happens. A warning is reported via the LibLogger whenever you forget this.
 *
 * @author Tim Niemueller
 * @see BlackBoardInterfaceManager::register_listener()
 * @see BlackBoardInterfaceManager::unregister_listener()
 */

/** Empty constructor. */
BlackBoardEventListener::BlackBoardEventListener()
{
}

/** Destructor. */
BlackBoardEventListener::~BlackBoardEventListener()
{
  char *tmp;

  while ( ! __bbel_data_interfaces.empty() ) {
    __bbel_ii = __bbel_data_interfaces.begin();
    tmp = (*__bbel_ii).first;
    __bbel_data_interfaces.erase(__bbel_ii);
    free(tmp);
  }

  while ( ! __bbel_reader_interfaces.empty() ) {
    __bbel_ii = __bbel_reader_interfaces.begin();
    tmp = (*__bbel_ii).first;
    __bbel_reader_interfaces.erase(__bbel_ii);
    free(tmp);
  }

  while ( ! __bbel_writer_interfaces.empty() ) {
    __bbel_ii = __bbel_writer_interfaces.begin();
    tmp = (*__bbel_ii).first;
    __bbel_writer_interfaces.erase(__bbel_ii);
    free(tmp);
  }

  while ( ! __bbel_interface_create_types.empty() ) {
    __bbel_iti = __bbel_interface_create_types.begin();
    tmp = *__bbel_iti;
    __bbel_interface_create_types.erase(__bbel_iti);
    free(tmp);
  }

}


/** BlackBoard data changed notification.
 * This is called whenever the data in an interface that you registered for is
 * modified. This happens if a writer calls the Interface::write() method.
 * @param interface interface instance that you supplied to bbel_add_data_interface()
 */
void
BlackBoardEventListener::bb_data_changed(Interface *interface) throw()
{
}


/** BlackBoard interface created notification.
 * This is called whenever an interface is created for a type that you registered
 * for.
 * @param type type of the interface. If you want to store this make a copy as it
 * is not guaranteed that the supplied string exists for longer than the duration
 * of the method call
 * @param id ID of the newly created interface. If you want to store this make a
 * copy as it is not guaranteed that the supplied string exists for longer than
 * the duration of the method call
 */
void
BlackBoardEventListener::bb_interface_created(const char *type, const char *id ) throw()
{
}


/** A reading instance has been opened for a watched interface.
 * This is called whenever a reading instance of the interface you are watching
 * is opened.
 * @param interface interface instance that you supplied to bbel_add_reader_interface()
 */
void
BlackBoardEventListener::bb_interface_reader_added(Interface *interface) throw()
{
}


/** A reading instance has been closed for a watched interface.
 * This is called whenever a reading instance of an interface you are watching
 * is closed.
 * @param interface interface instance that you supplied to bbel_add_reader_interface()
 */
void
BlackBoardEventListener::bb_interface_reader_removed(Interface *interface) throw()
{
}


/** A writing instance has been opened for a watched interface.
 * This is called whenever a writing instance of the interface you are watching
 * is opened.
 * @param interface interface instance that you supplied to bbel_add_writer_interface()
 */
void
BlackBoardEventListener::bb_interface_writer_added(Interface *interface) throw()
{
}


/** A writing instance has been closed for a watched interface.
 * This is called whenever a writing instance of an interface you are watching
 * is closed.
 * @param interface interface instance that you supplied to bbel_add_writer_interface()
 */
void
BlackBoardEventListener::bb_interface_writer_removed(Interface *interface) throw()
{
}


/** Add an interface to the data modification watch list.
 * @param interface interface to watch for data modifications.
 */
void
BlackBoardEventListener::bbel_add_data_interface(Interface *interface)
{
  if ( __bbel_data_interfaces.find((char *)interface->uid()) != __bbel_data_interfaces.end() ) {
    throw Exception("Interface %s already registered (data)", interface->uid());
  }
  __bbel_data_interfaces[strdup(interface->uid())] = interface;
}

/** Add an interface to the reader addition/removal watch list.
 * This method does not mean that you add interfaces that you opened for reading
 * but that you add an interface that you want to be informed for when reader
 * addition/removal happens.
 * @param interface interface to watch for addition/removal of readers
 */
void
BlackBoardEventListener::bbel_add_reader_interface(Interface *interface)
{
  if ( __bbel_reader_interfaces.find((char *)interface->uid()) != __bbel_reader_interfaces.end() ) {
    throw Exception("Interface %s already registered (reader)", interface->uid());
  }
  __bbel_reader_interfaces[strdup(interface->uid())] = interface;
}


/** Add an interface to the writer addition/removal watch list.
 * This method does not mean that you add interfaces that you opened for writing
 * but that you add an interface that you want to be informed for when writer
 * addition/removal happens.
 * @param interface interface to watch for addition/removal of writers
 */
void
BlackBoardEventListener::bbel_add_writer_interface(Interface *interface)
{
  if ( __bbel_writer_interfaces.find((char *)interface->uid()) != __bbel_writer_interfaces.end() ) {
    throw Exception("Interface %s already registered (writer)", interface->uid());
  }
  __bbel_writer_interfaces[strdup(interface->uid())] = interface;
}

/** Add interface creation type to watch list.
 * With this you add an interface type to the watch list. For any type on this list
 * you will be notified if an interface is created.
 * @param type type to watch
 */
void
BlackBoardEventListener::bbel_add_interface_create_type(const char *type) throw()
{
  __bbel_interface_create_types.insert(strdup(type));
}


/** Get data modification watch list.
 * @return data modification watch list
 */
BlackBoardEventListener::InterfaceLockHashMap *
BlackBoardEventListener::bbel_data_interfaces() throw()
{
  return &__bbel_data_interfaces;
}

/** Get reader watch list.
 * @return reader watch list
 */
BlackBoardEventListener::InterfaceLockHashMap *
BlackBoardEventListener::bbel_reader_interfaces() throw()
{
  return &__bbel_reader_interfaces;
}

/** Get writer watch list.
 * @return writer watch list
 */
BlackBoardEventListener::InterfaceLockHashMap *
BlackBoardEventListener::bbel_writer_interfaces() throw()
{
  return &__bbel_writer_interfaces;
}

/** Get interface type watch list.
 * @return interface type watch list
 */
BlackBoardEventListener::InterfaceTypeLockHashSet *
BlackBoardEventListener::bbel_interface_create_types() throw()
{
  return &__bbel_interface_create_types;
}


/** Get interface instance for given UID.
 * A data modification notification is about to be triggered. For this the
 * interface instance that has been added to the event listener is determined.
 * @param iuid interface unique ID
 * @return interface instance, NULL if not in list (non-fatal error)
 */
Interface *
BlackBoardEventListener::bbel_data_interface(const char *iuid) throw()
{
  __bbel_data_interfaces.lock();
  bool found = ((__bbel_ii = __bbel_data_interfaces.find((char *)iuid)) != __bbel_data_interfaces.end());
  __bbel_data_interfaces.unlock();
  if ( found ) {
    return (*__bbel_ii).second;
  } else {
    return NULL;
  }
}


/** Get interface instance for given UID.
 * A reader notification is about to be triggered. For this the
 * interface instance that has been added to the event listener is determined.
 * @param iuid interface unique ID
 * @return interface instance, NULL if not in list (non-fatal error)
 */
Interface *
BlackBoardEventListener::bbel_reader_interface(const char *iuid) throw()
{
  __bbel_reader_interfaces.lock();
  bool found = ((__bbel_ii = __bbel_reader_interfaces.find((char *)iuid)) != __bbel_reader_interfaces.end());
  __bbel_reader_interfaces.unlock();
  if ( found ) {
    return (*__bbel_ii).second;
  } else {
    return NULL;
  }
}


/** Get interface instance for given UID.
 * A writer notification is about to be triggered. For this the
 * interface instance that has been added to the event listener is determined.
 * @param iuid interface unique ID
 * @return interface instance, NULL if not in list (non-fatal error)
 */
Interface *
BlackBoardEventListener::bbel_writer_interface(const char *iuid) throw()
{
  __bbel_writer_interfaces.lock();
  bool found = ((__bbel_ii = __bbel_writer_interfaces.find((char *)iuid)) != __bbel_writer_interfaces.end());
  __bbel_writer_interfaces.unlock();
  if ( found ) {
    return (*__bbel_ii).second;
  } else {
    return NULL;
  }
}
