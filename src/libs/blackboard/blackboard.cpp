
/***************************************************************************
 *  blackboard.cpp - BlackBoard plugin
 *
 *  Generated: Sat Sep 16 17:11:13 2006 (on train to Cologne)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/blackboard.h>
#include <blackboard/bbconfig.h>
#include <blackboard/message_manager.h>
#include <blackboard/memory_manager.h>
#include <blackboard/interface_manager.h>
#include <blackboard/network_handler.h>
#include <blackboard/notifier.h>

#include <string>
#include <cstring>

/** @class BlackBoard <blackboard/blackboard.h>
 * The BlackBoard.
 * This class is the single one entry point for programs that use the BlackBoard.
 * It is used to open and close interfaces, register and unregister listeners and
 * observers and to maintain the BlackBoard shared memory segment. Not other classes
 * shall be used directly.
 *
 * The BlackBoard holds a number of so-called interfaces. The interfaces store
 * data and provide means to pass messages. The BlackBoard also allows for registering
 * listeners and observers. The listeners can be used to get events for specific
 * interfaces while the observer gets global interface creation and destruction
 * events for a specified set of types of interfaces.

 * An interface consists of a few parts. First there is the storage block. This
 * is a chunk of memory in the shared memory segment where the actual data is stored.
 * Then there is the accessor object, an instance of a derivate of the Interface
 * class which is used to access the data in the shared memory segment. Last but
 * not least there is an internal message queue that can be used to pass messages
 * from readers to the writer (not the other way around!).
 *
 * The interface manager keeps track of all the allocated interfaces. Events
 * can be triggered if a specific interface changes (like logging the data to
 * a file, sending it over the network or notifying another interface of such
 * a change).
 *
 * Interfaces can only be instantiated through the BlackBoard. The BlackBoard
 * instantiates an interface on request and guarantees that the instance
 * is fully initialized and usable. This cannot be guaranteed if instantiating an
 * interface through any other means!
 *
 * Interfaces can be opened for reading or writing, not both! There can be only
 * one writer at a time for any given interface. Interfaces are identified via a
 * type (which denotes the data and its semantics) and an identifier. There may
 * be several interfaces for a given type, but the identifier has to be unique.
 * The identifier is in most cases a well-known string that is used to share data
 * among plugins.
 *
 * Interfaces provide a way to propagate data to the writer via messages. Available
 * messages types depend on the interface type. Only matching messages are accepted
 * and can be queued.
 *
 * The BlackBoard can operate in two modes, master and slave. Only the master
 * creates and destroys the shared memory segment. Currently, the slave mode is not
 * fully implemented and thus may not be used.
 *
 * @see Interface
 * @see Message
 *
 * @author Tim Niemueller
 */

/** Data changed notification flag. */
const unsigned int BlackBoard::BBIL_FLAG_DATA      = 1;
/** Message received notification flag. */
const unsigned int BlackBoard::BBIL_FLAG_MESSAGES  = 2;
/** Reader added/removed notification flag. */
const unsigned int BlackBoard::BBIL_FLAG_READER    = 4;
/** Writer added/removed notification flag. */
const unsigned int BlackBoard::BBIL_FLAG_WRITER    = 8;

/** All interface listener notifications. */
const unsigned int BlackBoard::BBIL_FLAG_ALL = 
  BBIL_FLAG_DATA | BBIL_FLAG_MESSAGES | BBIL_FLAG_READER | BBIL_FLAG_WRITER;

/** Interface creation notification flag. */
const unsigned int BlackBoard::BBIO_FLAG_CREATED   = 1;

/** Interface destruction notification flag. */
const unsigned int BlackBoard::BBIO_FLAG_DESTROYED = 2;

/** All interface observer notifications */
const unsigned int BlackBoard::BBIO_FLAG_ALL =
  BBIO_FLAG_CREATED | BBIO_FLAG_DESTROYED;



/** Constructor.
 * @param master true to operate in master mode, false otherwise
 */
BlackBoard::BlackBoard(bool master)
{
  __memmgr = new BlackBoardMemoryManager(BLACKBOARD_MEMORY_SIZE,
					 BLACKBOARD_VERSION,
					 master,
					 BLACKBOARD_MAGIC_TOKEN);

  __notifier = new BlackBoardNotifier();
  __msgmgr = new BlackBoardMessageManager(__notifier);
  __im = new BlackBoardInterfaceManager(__memmgr, __msgmgr, __notifier);

  __msgmgr->set_interface_manager(__im);

  __nethandler = NULL;
}


/** Destructor. */
BlackBoard::~BlackBoard()
{
  if ( __nethandler ) {
    __nethandler->cancel();
    __nethandler->join();
    delete __nethandler;
  }
  delete __im;
  delete __msgmgr;
  delete __memmgr;
  delete __notifier;
}


/** Open interface for reading.
 * This will create a new interface instance of the given type. The result can be
 * casted to the appropriate type.
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 */
Interface *
BlackBoard::open_for_reading(const char *type, const char *identifier)
{
  try {
    return __im->open_for_reading(type, identifier);
  } catch (Exception &e) {
    throw;
  }
}


/** Open interface for writing.
 * This will create a new interface instance of the given type. The result can be
 * casted to the appropriate type. This will only succeed if there is not already
 * a writer for the given interface type/id!
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception BlackBoardWriterActiveException thrown if there is already a writing
 * instance with the same type/id
 */
Interface *
BlackBoard::open_for_writing(const char *type, const char *identifier)
{
  try {
    return __im->open_for_writing(type, identifier);
  } catch (Exception &e) {
    throw;
  }
}


/** Open all interfaces of the given type for reading.
 * This will create interface instances for all currently registered interfaces of
 * the given type. The result can be casted to the appropriate type.
 * @param type type of the interface
 * @param id_prefix if set only interfaces whose ids have this prefix are returned
 * @return list of new fully initialized interface instances of requested type. The
 * is allocated using new and you have to free it using delete after you are done
 * with it!
 */
std::list<Interface *> *
BlackBoard::open_all_of_type_for_reading(const char *type, const char *id_prefix)
{
  try {
    return __im->open_all_of_type_for_reading(type, id_prefix);
  } catch (Exception &e) {
    throw;
  }  
}


/** Close interface.
 * @param interface interface to close
 */
void
BlackBoard::close(Interface *interface)
{
  __im->close(interface);
}


/** Get list of interfaces.
 * @return list of interfaces
 */
InterfaceInfoList *
BlackBoard::list_all() const
{
  return __im->list_all();
}


/** Register BB event listener.
 * @param listener BlackBoard event listener to register
 * @param flags an or'ed combination of BBIL_FLAG_DATA, BBIL_FLAG_READER, BBIL_FLAG_WRITER
 * and BBIL_FLAG_INTERFACE. Only for the given types the event listener is registered.
 * BBIL_FLAG_ALL can be supplied to register for all events.
 */
void
BlackBoard::register_listener(BlackBoardInterfaceListener *listener, unsigned int flags)
{
  __notifier->register_listener(listener, flags);
}


/** Unregister BB interface listener.
 * This will remove the given BlackBoard interface listener from any event that it was
 * previously registered for.
 * @param listener BlackBoard event listener to remove
 */
void
BlackBoard::unregister_listener(BlackBoardInterfaceListener *listener)
{
  __notifier->unregister_listener(listener);
}


/** Register BB interface observer.
 * @param observer BlackBoard interface observer to register
 * @param flags an or'ed combination of BBIO_FLAG_CREATED, BBIO_FLAG_DESTROYED
 */
void
BlackBoard::register_observer(BlackBoardInterfaceObserver *observer, unsigned int flags)
{
  __notifier->register_observer(observer, flags);
}


/** Unregister BB interface observer.
 * This will remove the given BlackBoard event listener from any event that it was
 * previously registered for.
 * @param observer BlackBoard event listener to remove
 */
void
BlackBoard::unregister_observer(BlackBoardInterfaceObserver *observer)
{
  __notifier->unregister_observer(observer);
}


/** Get memory manager.
 * CAUTION: This is NOT meant to be used in your application.
 * This returns a pointer to the used memory manager. The return type
 * is declared const. Use this only for debugging purposes to output info about
 * the BlackBoard memory.
 * @return const pointer to memory manager
 */
const BlackBoardMemoryManager *
BlackBoard::memory_manager() const
{
  return __memmgr;
}


/** Start network handler.
 * This will start the network handler thread and register it with the given hub.
 * @param hub hub to use and to register with
 */
void
BlackBoard::start_nethandler(FawkesNetworkHub *hub)
{
  if ( __nethandler ) {
    throw Exception("BlackBoardNetworkHandler already started");
  }
  __nethandler = new BlackBoardNetworkHandler(this, hub);
  __nethandler->start();
}

