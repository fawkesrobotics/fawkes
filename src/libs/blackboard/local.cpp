
/***************************************************************************
 *  local.cpp - Local BlackBoard
 *
 *  Created: Sat Sep 16 17:11:13 2006 (on train to Cologne)
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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
#include <blackboard/internal/message_manager.h>
#include <blackboard/internal/memory_manager.h>
#include <blackboard/internal/interface_manager.h>
#include <blackboard/internal/notifier.h>
#include <blackboard/net/handler.h>

// for -C: bb_cleanup
#include <utils/ipc/shm.h>
#include <blackboard/shmem/header.h>
#include <blackboard/shmem/lister.h>

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


/** Shared Memory Constructor.
 * @param memsize size of memory in bytes
 * @param magic_token magic token used for shared memory segment
 * @param master true to operate in master mode, false otherwise
 */
LocalBlackBoard::LocalBlackBoard(size_t memsize,
				 const char *magic_token, bool master)
{
  __memmgr = new BlackBoardMemoryManager(memsize, BLACKBOARD_VERSION, master);

  __msgmgr = new BlackBoardMessageManager(__notifier);
  __im = new BlackBoardInterfaceManager(__memmgr, __msgmgr, __notifier);

  __msgmgr->set_interface_manager(__im);

  __nethandler = NULL;
}


/** Heap Memory Constructor.
 * @param memsize size of memory in bytes
 */
LocalBlackBoard::LocalBlackBoard(size_t memsize)
{
  __memmgr = new BlackBoardMemoryManager(memsize);

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
}


Interface *
LocalBlackBoard::open_for_reading(const char *type, const char *identifier, const char *owner)
{
  try {
    return __im->open_for_reading(type, identifier, owner);
  } catch (Exception &e) {
    throw;
  }
}


Interface *
LocalBlackBoard::open_for_writing(const char *type, const char *identifier, const char *owner)
{
  try {
    return __im->open_for_writing(type, identifier, owner);
  } catch (Exception &e) {
    throw;
  }
}


std::list<Interface *>
LocalBlackBoard::open_multiple_for_reading(const char *type_pattern,
					   const char *id_pattern,
					   const char *owner)
{
  try {
    return __im->open_multiple_for_reading(type_pattern, id_pattern, owner);
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


InterfaceInfoList *
LocalBlackBoard::list(const char *type_pattern, const char *id_pattern)
{
  return __im->list(type_pattern, id_pattern);
}


bool
LocalBlackBoard::is_alive() const throw()
{
  return true;
}


bool
LocalBlackBoard::try_aliveness_restore() throw()
{
  return true;
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
