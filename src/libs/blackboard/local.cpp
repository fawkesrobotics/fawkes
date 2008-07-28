
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

#include <blackboard/local.h>
#include <blackboard/bbconfig.h>
#include <blackboard/message_manager.h>
#include <blackboard/memory_manager.h>
#include <blackboard/interface_manager.h>
#include <blackboard/network_handler.h>
#include <blackboard/notifier.h>

// for -C: bb_cleanup
#include <utils/ipc/shm.h>
#include <blackboard/shmem_header.h>
#include <blackboard/shmem_lister.h>

#include <string>
#include <cstring>

namespace fawkes {

/** @class LocalBlackBoard <blackboard/local.h>
 * Local BlackBoard.
 *
 * @see Interface
 * @see Message
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param memsize size of memory in bytes
 * @param magic_token magic token used for shared memory segment
 * @param master true to operate in master mode, false otherwise
 */
LocalBlackBoard::LocalBlackBoard(size_t memsize,
				 const char *magic_token,
				 bool master)
{
  __memmgr = new BlackBoardMemoryManager(memsize, BLACKBOARD_VERSION,
					 master, magic_token);

  __notifier = new BlackBoardNotifier();
  __msgmgr = new BlackBoardMessageManager(__notifier);
  __im = new BlackBoardInterfaceManager(__memmgr, __msgmgr, __notifier);

  __msgmgr->set_interface_manager(__im);

  __nethandler = NULL;
}


/** Destructor. */
LocalBlackBoard::~LocalBlackBoard()
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


Interface *
LocalBlackBoard::open_for_reading(const char *type, const char *identifier)
{
  try {
    return __im->open_for_reading(type, identifier);
  } catch (Exception &e) {
    throw;
  }
}


Interface *
LocalBlackBoard::open_for_writing(const char *type, const char *identifier)
{
  try {
    return __im->open_for_writing(type, identifier);
  } catch (Exception &e) {
    throw;
  }
}


std::list<Interface *> *
LocalBlackBoard::open_all_of_type_for_reading(const char *type, const char *id_prefix)
{
  try {
    return __im->open_all_of_type_for_reading(type, id_prefix);
  } catch (Exception &e) {
    throw;
  }  
}


void
LocalBlackBoard::close(Interface *interface)
{
  __im->close(interface);
}


InterfaceInfoList *
LocalBlackBoard::list_all()
{
  return __im->list_all();
}


bool
LocalBlackBoard::is_alive() const throw()
{
  return true;
}


void
LocalBlackBoard::register_listener(BlackBoardInterfaceListener *listener, unsigned int flags)
{
  __notifier->register_listener(listener, flags);
}


void
LocalBlackBoard::unregister_listener(BlackBoardInterfaceListener *listener)
{
  __notifier->unregister_listener(listener);
}


void
LocalBlackBoard::register_observer(BlackBoardInterfaceObserver *observer, unsigned int flags)
{
  __notifier->register_observer(observer, flags);
}


void
LocalBlackBoard::unregister_observer(BlackBoardInterfaceObserver *observer)
{
  __notifier->unregister_observer(observer);
}


/** Cleanup orphaned BlackBoard segments.
 * This erase orphaned shared memory segments that belonged to a
 * BlackBoard.
 * @param magic_token magic token of shared memory segments
 * @param use_lister true to use a lister with console output
 */
void
LocalBlackBoard::cleanup(const char *magic_token, bool use_lister)
{
  BlackBoardSharedMemoryHeader *bbsh = new BlackBoardSharedMemoryHeader( BLACKBOARD_VERSION );
  BlackBoardSharedMemoryLister *bblister = NULL;
  if ( use_lister ) {
    bblister = new BlackBoardSharedMemoryLister();
  }
  SharedMemory::erase_orphaned(magic_token, bbsh, bblister);
  delete bblister;
  delete bbsh;
}


/** Get memory manager.
 * CAUTION: This is NOT meant to be used in your application.
 * This returns a pointer to the used memory manager. The return type
 * is declared const. Use this only for debugging purposes to output info about
 * the BlackBoard memory.
 * @return const pointer to memory manager
 */
const BlackBoardMemoryManager *
LocalBlackBoard::memory_manager() const
{
  return __memmgr;
}


/** Start network handler.
 * This will start the network handler thread and register it with the given hub.
 * @param hub hub to use and to register with
 */
void
LocalBlackBoard::start_nethandler(FawkesNetworkHub *hub)
{
  if ( __nethandler ) {
    throw Exception("BlackBoardNetworkHandler already started");
  }
  __nethandler = new BlackBoardNetworkHandler(this, hub);
  __nethandler->start();
}

} // end namespace fawkes
