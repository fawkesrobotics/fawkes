 
/***************************************************************************
 *  interface_listener.cpp - BlackBoard event listener
 *
 *  Created: Wed Nov 08 10:00:34 2007
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/interface_listener.h>
#include <interface/interface.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

namespace fawkes {

/** @class BlackBoardInterfaceListener <blackboard/interface_listener.h>
 * BlackBoard interface listener.
 * Derive this class if you want to be notified of specific BlackBoard 
 * events regarding instances of interfaces.
 *
 * The bb_interface_* methods are called during the appropriate operation. The
 * operation that you carry out in this event handler really has to damn fast, or
 * the performance of the whole system will suffer severely. For this reason use
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
 * You will not be notified if you change data of the interface that you registered
 * for or remove your own reading/writing instance of an interface.
 *
 * Here is a simple life cycle of a BlackBoard interface listener:
 * First you create your interface that you want to listen for.
 * The protected methods bbil_add_data_interface(), bbil_add_reader_interface(),
 * bbil_add_writer_interface() and bbil_add_interface_create_type() have to
 * be called with the appropriate interfaces <i>before</i> the event handler is
 * actually registered with the interface manager! From
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

/** Constructor.
 * @param name_format format of name to identify the listener,
 * see sprintf for supported tokens
 */
  BlackBoardInterfaceListener::BlackBoardInterfaceListener(const char *name_format, ...)
{
  va_list arg;
  va_start(arg, name_format);
  vasprintf(&__name, name_format, arg);
  va_end(arg);
}


/** Destructor. */
BlackBoardInterfaceListener::~BlackBoardInterfaceListener()
{
  while ( ! __bbil_data_interfaces.empty() ) {
    __bbil_ii = __bbil_data_interfaces.begin();
    __bbil_data_interfaces.erase(__bbil_ii);
  }

  while ( ! __bbil_reader_interfaces.empty() ) {
    __bbil_ii = __bbil_reader_interfaces.begin();
    __bbil_reader_interfaces.erase(__bbil_ii);
  }

  while ( ! __bbil_writer_interfaces.empty() ) {
    __bbil_ii = __bbil_writer_interfaces.begin();
    __bbil_writer_interfaces.erase(__bbil_ii);
  }

  free(__name);
}


/** Get BBIL name.
 * @return BBIL name
 */
const char *
BlackBoardInterfaceListener::bbil_name() const
{
  return __name;
}


/** BlackBoard data changed notification.
 * This is called whenever the data in an interface that you registered for is
 * modified. This happens if a writer calls the Interface::write() method.
 * @param interface interface instance that you supplied to bbil_add_data_interface()
 */
void
BlackBoardInterfaceListener::bb_interface_data_changed(Interface *interface) throw()
{
}


/** BlackBoard message received notification.
 * This is called whenever a message is received for this interface. This method is
 * only called for writing instances of an interface, never on reading instances.
 * If you have processed the message already, you can order that the message is not
 * enqueued by returning false. Returning true will enqueue the message as usual.
 * You should only do very (very!) quick tasks directly in this method, as it is
 * out of the regular thread context and can harm performance of other plugins and
 * the system as a whole. Note that if you decide to return false the message is
 * not referenced. If you want to keep it longer you have to ref() it by yourself.
 * An example where this would really make sense is a "STOP" message for the motor,
 * which needs to be processed ASAP and maybe even waiting a couple of miliseconds
 * for the next cycle is not acceptable.
 * @param interface interface instance that you supplied to bbil_add_message_interface()
 * @param message the message that was sent
 * @return true to get the message enqueued afterwards as usual, false to prevent
 * queuing of the message.
 */
bool
BlackBoardInterfaceListener::bb_interface_message_received(Interface *interface,
							   Message *message) throw()
{
  return true;
}


/** A reading instance has been opened for a watched interface.
 * This is called whenever a reading instance of the interface you are watching
 * is opened.
 * @param interface interface instance that you supplied to bbil_add_reader_interface()
 * @param instance_serial the instance serial of the reading instance that has just been
 * added.
 */
void
BlackBoardInterfaceListener::bb_interface_reader_added(Interface *interface,
						       unsigned int instance_serial) throw()
{
}


/** A reading instance has been closed for a watched interface.
 * This is called whenever a reading instance of an interface you are watching
 * is closed.
 * @param interface interface instance that you supplied to bbil_add_reader_interface()
 * @param instance_serial the instance serial of the reading instance that has just been
 * removed.
 */
void
BlackBoardInterfaceListener::bb_interface_reader_removed(Interface *interface,
							 unsigned int instance_serial) throw()
{
}


/** A writing instance has been opened for a watched interface.
 * This is called whenever a writing instance of the interface you are watching
 * is opened.
 * @param interface interface instance that you supplied to bbil_add_writer_interface()
 * @param instance_serial the instance serial of the writing instance that has just been
 * added.
 */
void
BlackBoardInterfaceListener::bb_interface_writer_added(Interface *interface,
						       unsigned int instance_serial) throw()
{
}


/** A writing instance has been closed for a watched interface.
 * This is called whenever a writing instance of an interface you are watching
 * is closed.
 * @param interface interface instance that you supplied to bbil_add_writer_interface()
 * @param instance_serial the instance serial of the writing instance that has just been
 * removed.
 */
void
BlackBoardInterfaceListener::bb_interface_writer_removed(Interface *interface,
							 unsigned int instance_serial) throw()
{
}


/** Add an interface to the data modification watch list.
 * @param interface interface to watch for data modifications.
 */
void
BlackBoardInterfaceListener::bbil_add_data_interface(Interface *interface)
{
  if ( __bbil_data_interfaces.find((char *)interface->uid()) != __bbil_data_interfaces.end() ) {
    throw Exception("Interface %s already registered (data)", interface->uid());
  }
  __bbil_data_interfaces[interface->uid()] = interface;
}

/** Add an interface to the message received watch list.
 * @param interface interface to watch for messages
 */
void
BlackBoardInterfaceListener::bbil_add_message_interface(Interface *interface)
{
  if ( ! interface->is_writer() ) {
    throw Exception("Message received events can only be watched by writing instances");
  }
  if ( __bbil_message_interfaces.find((char *)interface->uid()) != __bbil_message_interfaces.end() ) {
    throw Exception("Interface %s already registered (message)", interface->uid());
  }
  __bbil_message_interfaces[interface->uid()] = interface;
}


/** Add an interface to the reader addition/removal watch list.
 * This method does not mean that you add interfaces that you opened for reading
 * but that you add an interface that you want to be informed for when reader
 * addition/removal happens.
 * @param interface interface to watch for addition/removal of readers
 */
void
BlackBoardInterfaceListener::bbil_add_reader_interface(Interface *interface)
{
  if ( __bbil_reader_interfaces.find((char *)interface->uid()) != __bbil_reader_interfaces.end() ) {
    throw Exception("Interface %s already registered (reader)", interface->uid());
  }
  __bbil_reader_interfaces[interface->uid()] = interface;
}


/** Add an interface to the writer addition/removal watch list.
 * This method does not mean that you add interfaces that you opened for writing
 * but that you add an interface that you want to be informed for when writer
 * addition/removal happens.
 * @param interface interface to watch for addition/removal of writers
 */
void
BlackBoardInterfaceListener::bbil_add_writer_interface(Interface *interface)
{
  if ( __bbil_writer_interfaces.find((char *)interface->uid()) != __bbil_writer_interfaces.end() ) {
    throw Exception("Interface %s already registered (writer)", interface->uid());
  }
  __bbil_writer_interfaces[interface->uid()] = interface;
}


/** Get data modification watch list.
 * @return data modification watch list
 */
BlackBoardInterfaceListener::InterfaceLockMap *
BlackBoardInterfaceListener::bbil_data_interfaces() throw()
{
  return &__bbil_data_interfaces;
}

/** Get message received watch list.
 * @return message received watch list
 */
BlackBoardInterfaceListener::InterfaceLockMap *
BlackBoardInterfaceListener::bbil_message_interfaces() throw()
{
  return &__bbil_message_interfaces;
}

/** Get reader watch list.
 * @return reader watch list
 */
BlackBoardInterfaceListener::InterfaceLockMap *
BlackBoardInterfaceListener::bbil_reader_interfaces() throw()
{
  return &__bbil_reader_interfaces;
}

/** Get writer watch list.
 * @return writer watch list
 */
BlackBoardInterfaceListener::InterfaceLockMap *
BlackBoardInterfaceListener::bbil_writer_interfaces() throw()
{
  return &__bbil_writer_interfaces;
}


/** Get interface instance for given UID.
 * A data modification notification is about to be triggered. For this the
 * interface instance that has been added to the event listener is determined.
 * @param iuid interface unique ID
 * @return interface instance, NULL if not in list (non-fatal error)
 */
Interface *
BlackBoardInterfaceListener::bbil_data_interface(const char *iuid) throw()
{
  __bbil_data_interfaces.lock();
  bool found = ((__bbil_ii = __bbil_data_interfaces.find((char *)iuid)) != __bbil_data_interfaces.end());
  __bbil_data_interfaces.unlock();
  if ( found ) {
    return (*__bbil_ii).second;
  } else {
    return NULL;
  }
}


/** Get interface instance for given UID.
 * A message received notification is about to be triggered. For this the
 * interface instance that has been added to the event listener is determined.
 * @param iuid interface unique ID
 * @return interface instance, NULL if not in list (non-fatal error)
 */
Interface *
BlackBoardInterfaceListener::bbil_message_interface(const char *iuid) throw()
{
  __bbil_message_interfaces.lock();
  bool found = ((__bbil_ii = __bbil_message_interfaces.find((char *)iuid)) != __bbil_message_interfaces.end());
  __bbil_message_interfaces.unlock();
  if ( found ) {
    return (*__bbil_ii).second;
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
BlackBoardInterfaceListener::bbil_reader_interface(const char *iuid) throw()
{
  __bbil_reader_interfaces.lock();
  bool found = ((__bbil_ii = __bbil_reader_interfaces.find((char *)iuid)) != __bbil_reader_interfaces.end());
  __bbil_reader_interfaces.unlock();
  if ( found ) {
    return (*__bbil_ii).second;
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
BlackBoardInterfaceListener::bbil_writer_interface(const char *iuid) throw()
{
  __bbil_writer_interfaces.lock();
  bool found = ((__bbil_ii = __bbil_writer_interfaces.find((char *)iuid)) != __bbil_writer_interfaces.end());
  __bbil_writer_interfaces.unlock();
  if ( found ) {
    return (*__bbil_ii).second;
  } else {
    return NULL;
  }
}

} // end namespace fawkes
