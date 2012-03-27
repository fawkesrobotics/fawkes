 
/***************************************************************************
 *  interface_listener.cpp - BlackBoard event listener
 *
 *  Created: Wed Nov 08 10:00:34 2007
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
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
#include <core/exceptions/system.h>
#include <core/threading/mutex_locker.h>
#include <interface/interface.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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
  if (vasprintf(&__name, name_format, arg) == -1) {
    throw OutOfMemoryException("BlackBoardInterfaceListener ctor: vasprintf() failed");
  }
  va_end(arg);

  __bbil_queue_mutex = new Mutex();
  __bbil_maps_mutex = new Mutex();
}


/** Destructor. */
BlackBoardInterfaceListener::~BlackBoardInterfaceListener()
{
  free(__name);

  delete __bbil_queue_mutex;
  delete __bbil_maps_mutex;
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


void
BlackBoardInterfaceListener::bbil_queue_add(QueueEntryType type, bool op,
                                            InterfaceMap &not_in_map,
                                            Interface *interface,
                                            const char *hint)
{
  MutexLocker lock(__bbil_queue_mutex);

  if (op) {
    if (not_in_map.find(interface->uid()) != not_in_map.end() ) {
      throw Exception("Interface %s already registered (%s)",
                      interface->uid(), hint);
    }
  }
  InterfaceQueue::iterator i;
  for (i = __bbil_queue.begin(); i != __bbil_queue.end(); ++i) {
    if ((i->type == type) && (*(i->interface) == *interface)) {
      __bbil_queue.erase(i);
      break;
    }
  }
  QueueEntry qe = { type, op, interface };
  __bbil_queue.push_back(qe);
}


/** Add an interface to the data modification watch list.
 * @param interface interface to watch for data modifications.
 */
void
BlackBoardInterfaceListener::bbil_add_data_interface(Interface *interface)
{
  bbil_queue_add(DATA, true, __bbil_maps.data, interface, "data");
}

/** Add an interface to the message received watch list.
 * @param interface interface to watch for messages
 */
void
BlackBoardInterfaceListener::bbil_add_message_interface(Interface *interface)
{
  if ( ! interface->is_writer() ) {
    throw Exception("Message received events can only be watched "
                    "on writing interface instances (%s)", interface->uid());
  }
  bbil_queue_add(MESSAGES, true, __bbil_maps.messages, interface, "messages");
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
  bbil_queue_add(READER, true, __bbil_maps.reader, interface, "reader");
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
  bbil_queue_add(WRITER, true, __bbil_maps.writer, interface, "writer");
}



/** Remove an interface to the data modification watch list.
 * Only remove interfaces from the list when not currently registered to
 * the BlackBoard or chaos and confusion will come upon you.
 * @param interface interface to watch for data modifications.
 */
void
BlackBoardInterfaceListener::bbil_remove_data_interface(Interface *interface)
{
  bbil_queue_add(DATA, false, __bbil_maps.data, interface, "data");
}

/** Remove an interface to the message received watch list.
 * Only remove interfaces from the list when not currently registered to
 * the BlackBoard or chaos and confusion will come upon you.
 * @param interface interface to watch for messages
 */
void
BlackBoardInterfaceListener::bbil_remove_message_interface(Interface *interface)
{
  bbil_queue_add(MESSAGES, false, __bbil_maps.messages, interface, "messages");
}


/** Remove an interface to the reader addition/removal watch list.
 * Only remove interfaces from the list when not currently registered to
 * the BlackBoard or chaos and confusion will come upon you.
 * @param interface interface to watch for addition/removal of readers
 */
void
BlackBoardInterfaceListener::bbil_remove_reader_interface(Interface *interface)
{
  bbil_queue_add(READER, false, __bbil_maps.reader, interface, "reader");
}


/** Remove an interface to the writer addition/removal watch list.
 * Only remove interfaces from the list when not currently registered to
 * the BlackBoard or chaos and confusion will come upon you.
 * @param interface interface to watch for addition/removal of writers
 */
void
BlackBoardInterfaceListener::bbil_remove_writer_interface(Interface *interface)
{
  bbil_queue_add(WRITER, false, __bbil_maps.writer, interface, "writer");
}


const BlackBoardInterfaceListener::InterfaceQueue &
BlackBoardInterfaceListener::bbil_acquire_queue() throw()
{
  __bbil_queue_mutex->lock();
  return __bbil_queue;
}

void
BlackBoardInterfaceListener::bbil_release_queue(BlackBoard::ListenerRegisterFlag flag) throw()
{
  __bbil_maps_mutex->lock();

  InterfaceQueue::iterator i = __bbil_queue.begin();
  while (i != __bbil_queue.end()) {
    if (i->op) { // add
      switch (i->type) {
      case DATA:
        if (flag & BlackBoard::BBIL_FLAG_DATA) {
          __bbil_maps.data[i->interface->uid()] = i->interface;
          i = __bbil_queue.erase(i);
        } else ++i;
        break;

      case MESSAGES:
        if (flag & BlackBoard::BBIL_FLAG_MESSAGES) {
          __bbil_maps.messages[i->interface->uid()] = i->interface;
          i = __bbil_queue.erase(i);
        } else ++i;
        break;

      case READER:
        if (flag & BlackBoard::BBIL_FLAG_READER) {
          __bbil_maps.reader[i->interface->uid()] = i->interface;
          i = __bbil_queue.erase(i);
        } else ++i;
        break;

      case WRITER:
        if (flag & BlackBoard::BBIL_FLAG_WRITER) {
          __bbil_maps.writer[i->interface->uid()] = i->interface;
          i = __bbil_queue.erase(i);
        } else ++i;
        break;

      default:
        ++i;
        break;
      }
    } else { // remove
      switch (i->type) {
      case DATA:
        if (flag & BlackBoard::BBIL_FLAG_DATA) {
          __bbil_maps.data.erase(i->interface->uid());
          i = __bbil_queue.erase(i);
        } else ++i;
        break;

      case MESSAGES:
        if (flag & BlackBoard::BBIL_FLAG_MESSAGES) {
          __bbil_maps.messages.erase(i->interface->uid());
          i = __bbil_queue.erase(i);
        } else ++i;
        break;

      case READER:
        if (flag & BlackBoard::BBIL_FLAG_READER) {
          __bbil_maps.reader.erase(i->interface->uid());
          i = __bbil_queue.erase(i);
        } else ++i;
        break;

      case WRITER:
        if (flag & BlackBoard::BBIL_FLAG_WRITER) {
          __bbil_maps.writer.erase(i->interface->uid());
          i = __bbil_queue.erase(i);
        } else ++i;
        break;

      default:
        ++i;
        break;
      }
    }
  }

  __bbil_maps_mutex->unlock();
  __bbil_queue_mutex->unlock();
}


const BlackBoardInterfaceListener::InterfaceMaps &
BlackBoardInterfaceListener::bbil_acquire_maps() throw()
{
  __bbil_maps_mutex->lock();
  return __bbil_maps;
}

void
BlackBoardInterfaceListener::bbil_release_maps() throw()
{
  __bbil_queue_mutex->lock();

  InterfaceMap::iterator i;
  for (i = __bbil_maps.data.begin(); i != __bbil_maps.data.end(); ++i) {
    QueueEntry qe = { DATA, true, i->second };
    __bbil_queue.push_back(qe);
  }
  for (i = __bbil_maps.messages.begin(); i != __bbil_maps.messages.end(); ++i) {
    QueueEntry qe = { MESSAGES, true, i->second };
    __bbil_queue.push_back(qe);
  }
  for (i = __bbil_maps.reader.begin(); i != __bbil_maps.reader.end(); ++i) {
    QueueEntry qe = { READER, true, i->second };
    __bbil_queue.push_back(qe);
  }
  for (i = __bbil_maps.writer.begin(); i != __bbil_maps.writer.end(); ++i) {
    QueueEntry qe = { WRITER, true, i->second };
    __bbil_queue.push_back(qe);
  }

  __bbil_maps.data.clear();
  __bbil_maps.messages.clear();
  __bbil_maps.reader.clear();
  __bbil_maps.writer.clear();

  __bbil_queue_mutex->unlock();
  __bbil_maps_mutex->unlock();
}


Interface *
BlackBoardInterfaceListener::bbil_find_interface(const char *iuid,
                                                 InterfaceMap &map)
{
  MutexLocker lock(__bbil_maps_mutex);
  InterfaceMap::iterator i;
  if ((i = map.find((char *)iuid)) != map.end()) {
    return i->second;
  } else {
    return NULL;
  }
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
  return bbil_find_interface(iuid, __bbil_maps.data);
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
  return bbil_find_interface(iuid, __bbil_maps.messages);
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
  return bbil_find_interface(iuid, __bbil_maps.reader);
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
  return bbil_find_interface(iuid, __bbil_maps.writer);
}

} // end namespace fawkes
