 
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __BLACKBOARD_INTERFACE_MANAGER_H_
#define __BLACKBOARD_INTERFACE_MANAGER_H_

#include <interfaces/mediators/interface_mediator.h>
#include <interfaces/interface.h>
#include <core/exceptions/software.h>
#include <typeinfo>
#include <map>

class BlackBoardMemoryManager;
class BlackBoardInternalsInterface;
class BlackBoardMessageManager;
class Mutex;
class Module;

class BlackBoardInterfaceManager : public InterfaceMediator
{
 friend class BlackBoardMessageManager;
 public:

  BlackBoardInterfaceManager(bool bb_master = false);
  virtual ~BlackBoardInterfaceManager();

  Interface *  openForReading(const char *interface_type, const char *identifier);
  Interface *  openForWriting(const char *interface_type, const char *identifier);
  void         close(Interface *interface);

  virtual bool existsWriter(const Interface *interface) const;
  virtual void notifyOfDataChange(const Interface *interface);

  template <class InterfaceType>
    InterfaceType * openForReading(const char *identifier);

  template <class InterfaceType>
    InterfaceType * openForWriting(const char *identifier);

  const BlackBoardMemoryManager *  getMemoryManager() const;

 private:
  Interface *  newInterfaceInstance(const char *type, const char *identifier);
  void         deleteInterfaceInstance(Interface *interface);

  void *       findInterfaceInMemory(const char *type, const char *identifier);
  unsigned int getNextMemSerial();
  unsigned int getNextInstanceSerial();
  void         createInterface(const char *type, const char *identifier,
			       Interface* &interface, void* &ptr);

  char *       stripClassType(const char *type);

  Interface *  getWriterForMemSerial(unsigned int mem_serial);

  BlackBoardInternalsInterface * openInternalsNonMaster();

 private:
  bool                          bb_master;
  BlackBoardMemoryManager      *memmgr;
  BlackBoardMessageManager     *msgmgr;
  Mutex                        *mutex;
  Module                       *iface_module;
  BlackBoardInternalsInterface *internals;

  std::map< unsigned int, Interface * >  writer_interfaces;
  std::map< unsigned int, RefCountRWLock * >  rwlocks;
};


/** Get interface of given type.
 * This will open a new interface for reading just like the non-template version of
 * openForReading(). But with the template method you will get a correctly typed object
 * that you can use. An TypeMismatchException is thrown if the string representation
 * of the type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception TypeMismatchException thrown if type in interface_type and the actual class
 * type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoardInterfaceManager::openForReading(const char *identifier)
{
  char *type_name = stripClassType(typeid(InterfaceType).name());
  InterfaceType *interface = dynamic_cast<InterfaceType *>(openForReading(type_name, identifier));
  delete[] type_name;
  if ( interface == 0 ) {
    throw TypeMismatchException("Interface (R) types do not match");
  } else {
    return interface;
  }
}



/** Get writer interface of given type.
 * This will open a new interface for writing just like the non-template version of
 * openForWriting(). But with the template method you will get a correctly typed object
 * that you can use. An TypeMismatchException is thrown if the string representation
 * of the type and the actual class type of the interface do not match.
 * @param identifier identifier of the interface
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception BlackBoardWriterActiveException thrown if there is already a writing
 * instance with the same type/id
 * @exception TypeMismatchException thrown if type in interface_type and the actual class
 * type do not fit.
 */
template <class InterfaceType>
InterfaceType *
BlackBoardInterfaceManager::openForWriting(const char *identifier)
{
  char *type_name = stripClassType(typeid(InterfaceType).name());
  InterfaceType *interface;
  try {
    interface = dynamic_cast<InterfaceType *>(openForWriting(type_name, identifier));
  } catch (Exception &e) {
    // just caught to properly free memory
    delete[] type_name;
    throw;
  }
  delete[] type_name;
  if ( interface == 0 ) {
    throw TypeMismatchException("Interface (W) types do not match");
  } else {
    return interface;
  }
}

#endif
