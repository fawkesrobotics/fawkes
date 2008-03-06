 
/***************************************************************************
 *  interface_manager.h - BlackBoard interface manager
 *
 *  Created: Mon Oct 09 19:05:46 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_INTERFACE_MANAGER_H_
#define __BLACKBOARD_INTERFACE_MANAGER_H_

#include <interface/mediators/interface_mediator.h>

#include <core/utils/lock_map.h>
#include <utils/misc/string_compare.h>

#include <list>

class Interface;
class InterfaceInfoList;
class BlackBoardMemoryManager;
class BlackBoardMessageManager;
class Mutex;
class BlackBoardInstanceFactory;
class BlackBoardInterfaceListener;
class BlackBoardInterfaceObserver;
class BlackBoardNotifier;
class RefCountRWLock;

class BlackBoardInterfaceManager : public InterfaceMediator
{
 friend class BlackBoardMessageManager;
 public:

  BlackBoardInterfaceManager(BlackBoardMemoryManager *bb_memmgr,
			     BlackBoardMessageManager *bb_msgmgr,
			     BlackBoardNotifier *bb_notifier);
  virtual ~BlackBoardInterfaceManager();

  Interface *  open_for_reading(const char *interface_type, const char *identifier);
  Interface *  open_for_writing(const char *interface_type, const char *identifier);
  void         close(Interface *interface);

  InterfaceInfoList *  list_all() const;

  std::list<Interface *> *  open_all_of_type_for_reading(const char *interface_type,
							 const char *id_prefix = NULL);

  /* InterfaceMediator methods */
  virtual bool exists_writer(const Interface *interface) const;
  virtual unsigned int num_readers(const Interface *interface) const;
  virtual void notify_of_data_change(const Interface *interface);

 private:
  const BlackBoardMemoryManager *  memory_manager() const;

  Interface *  new_interface_instance(const char *type, const char *identifier);
  void         delete_interface_instance(Interface *interface);

  void *       find_interface_in_memory(const char *type, const char *identifier);
  unsigned int next_mem_serial();
  unsigned int next_instance_serial();
  void         create_interface(const char *type, const char *identifier,
				Interface* &interface, void* &ptr);

  Interface *  writer_for_mem_serial(unsigned int mem_serial);

 private:
  unsigned int                  instance_serial;

  BlackBoardMemoryManager      *memmgr;
  BlackBoardMessageManager     *msgmgr;
  Mutex                        *mutex;
  BlackBoardInstanceFactory    *instance_factory;
  BlackBoardNotifier           *notifier;

  LockMap< unsigned int, Interface * >              writer_interfaces;
  LockMap< unsigned int, RefCountRWLock * >         rwlocks;
};

#endif
