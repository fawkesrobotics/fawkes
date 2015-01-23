
/***************************************************************************
 *  local.h - Local BlackBoard
 *
 *  Created: Sat Sep 16 17:09:15 2006 (on train to Cologne)
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

#ifndef __BLACKBOARD_LOCAL_H_
#define __BLACKBOARD_LOCAL_H_

#include <blackboard/blackboard.h>
#include <core/exceptions/software.h>

#include <list>

namespace fawkes {

class BlackBoardInterfaceManager;
class BlackBoardMemoryManager;
class BlackBoardMessageManager;
class BlackBoardNetworkHandler;
class BlackBoardNotifier;
class Interface;
class InterfaceInfoList;
class BlackBoardInterfaceListener;
class BlackBoardInterfaceObserver;
class FawkesNetworkHub;

class LocalBlackBoard : public BlackBoard
{
 public:
  LocalBlackBoard(size_t memsize);
  LocalBlackBoard(size_t memsize, const char *magic_token,
		  bool master = true);
  virtual ~LocalBlackBoard();

  virtual Interface *  open_for_reading(const char *interface_type,
					const char *identifier,
					const char *owner = NULL);
  virtual Interface *  open_for_writing(const char *interface_type,
					const char *identifier,
					const char *owner = NULL);
  virtual void         close(Interface *interface);

  virtual InterfaceInfoList *  list_all();
  virtual InterfaceInfoList *  list(const char *type_pattern,
				    const char *id_pattern);
  virtual bool                 is_alive() const throw();
  virtual bool                 try_aliveness_restore() throw();

  virtual std::list<Interface *>
    open_multiple_for_reading(const char *type_pattern,
			      const char *id_pattern = "*",
			      const char *owner = NULL);

  virtual void start_nethandler(FawkesNetworkHub *hub);

  static void cleanup(const char *magic_token, bool use_lister = false);

  /* for debugging only */
  const BlackBoardMemoryManager * memory_manager() const;

 private: /* members */
  BlackBoardInterfaceManager *__im;
  BlackBoardMemoryManager    *__memmgr;
  BlackBoardMessageManager   *__msgmgr;
  BlackBoardNetworkHandler   *__nethandler;
};


} // end namespace fawkes

#endif
