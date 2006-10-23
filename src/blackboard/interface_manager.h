 
/***************************************************************************
 *  interface_manager.h - BlackBoard interface manager
 *
 *  Generated: Mon Oct 09 19:05:46 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __BLACKBOARD_INTERFACE_MANAGER_H_
#define __BLACKBOARD_INTERFACE_MANAGER_H_

#include <interfaces/mediators/interface_mediator.h>
#include <interfaces/interface.h>

class BlackBoardMemoryManager;
class BlackBoardInternalsInterface;
class Mutex;
class Module;

class BlackBoardInterfaceManager : public InterfaceMediator
{
 public:

  BlackBoardInterfaceManager(BlackBoardMemoryManager *memmgr, bool bb_master = false);
  virtual ~BlackBoardInterfaceManager();

  Interface *  openForReading(const char *interface_type, const char *identifier);
  Interface *  openForWriting(const char *interface_type, const char *identifier);
  void         close(Interface *interface);

  virtual bool existsWriter(const Interface *interface) const;
  virtual void notifyOfDataChange(const Interface *interface);

 private:
  Interface *  newInterfaceInstance(const char *type);
  void         deleteInterfaceInstance(Interface *interface);

  void *       findInterfaceInMemory(const char *type, const char *identifier);
  unsigned int getNextMemSerial();
  unsigned int getNextInstanceSerial();
  void         createInterface(const char *type, const char *identifier,
			       Interface* &interface, void* &ptr);

  Interface *  open(const char *interface_type, const char *identifier);

  BlackBoardInternalsInterface * openInternalsNonMaster();


 private:
  bool                     bb_master;
  BlackBoardMemoryManager *memmgr;
  Mutex                   *mutex;
  Module                  *iface_module;
  BlackBoardInternalsInterface *internals;

};

#endif
