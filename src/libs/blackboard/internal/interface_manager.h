 
/***************************************************************************
 *  interface_manager.h - BlackBoard interface manager
 *
 *  Created: Mon Oct 09 19:05:46 2006
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

#ifndef __BLACKBOARD_INTERFACE_MANAGER_H_
#define __BLACKBOARD_INTERFACE_MANAGER_H_

#include <interface/mediators/interface_mediator.h>

#include <core/utils/lock_map.h>
#include <utils/misc/string_compare.h>

#include <list>
#include <string>

namespace fawkes {

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
 friend BlackBoardMessageManager;
 public:

  BlackBoardInterfaceManager(BlackBoardMemoryManager *bb_memmgr,
			     BlackBoardMessageManager *bb_msgmgr,
			     BlackBoardNotifier *bb_notifier);
  virtual ~BlackBoardInterfaceManager();

  Interface *  open_for_reading(const char *interface_type, const char *identifier,
				const char *owner = NULL);
  Interface *  open_for_writing(const char *interface_type, const char *identifier,
				const char *owner = NULL);
  void         close(Interface *interface);

  InterfaceInfoList *  list_all() const;
  InterfaceInfoList *  list(const char *type_pattern,
			    const char *id_pattern) const;

  std::list<Interface *> open_multiple_for_reading(const char *type_pattern,
						   const char *id_pattern = "*",
						   const char *owner = NULL);

  /* InterfaceMediator methods */
  virtual bool exists_writer(const Interface *interface) const;
  virtual unsigned int num_readers(const Interface *interface) const;
  virtual void notify_of_data_change(const Interface *interface);
  virtual std::list<std::string>  readers(const Interface *interface) const;
  virtual std::string             writer(const Interface *interface) const;

  std::list<std::string>  readers(const std::string &uid) const;
  std::string             writer(const std::string &uid) const;

 private:
  const BlackBoardMemoryManager *  memory_manager() const;

  Interface *  new_interface_instance(const char *type, const char *identifier, const char *owner);
  void         delete_interface_instance(Interface *interface);

  void *       find_interface_in_memory(const char *type, const char *identifier);
  unsigned int next_mem_serial();
  unsigned int next_instance_serial();
  void         create_interface(const char *type, const char *identifier, const char *owner,
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

  typedef struct _OwnerInfo {
    _OwnerInfo() : writer(NULL) {}
    Interface              *writer;
    std::list<Interface *>  readers;
  } OwnerInfo;
  LockMap<std::string, OwnerInfo> owner_info_;
};

} // end namespace fawkes

#endif
