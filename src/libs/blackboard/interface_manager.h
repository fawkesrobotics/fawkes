 
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

#include <interface/mediators/interface_mediator.h>
#include <interface/interface.h>
#include <core/exceptions/software.h>
#include <typeinfo>
#include <core/utils/lock_list.h>
#include <core/utils/lock_map.h>
#include <core/utils/lock_hashmap.h>
#include <utils/misc/string_compare.h>

class BlackBoardEventListener;
class BlackBoardMemoryManager;
class BlackBoardMessageManager;
class Mutex;
class Module;

class BlackBoardInterfaceManager : public InterfaceMediator
{
 friend class BlackBoardMessageManager;
 public:

  BlackBoardInterfaceManager(bool bb_master = false);
  virtual ~BlackBoardInterfaceManager();

  Interface *  open_for_reading(const char *interface_type, const char *identifier);
  Interface *  open_for_writing(const char *interface_type, const char *identifier);
  void         close(Interface *interface);

  std::list<Interface *> *  open_all_of_type_for_reading(const char *interface_type,
							 const char *id_prefix = NULL);
  template <class InterfaceType>
    std::list<InterfaceType *> *  open_all_of_type_for_reading(const char *id_prefix = NULL);

  virtual bool exists_writer(const Interface *interface) const;
  virtual void notify_of_data_change(const Interface *interface);

  template <class InterfaceType>
    InterfaceType * open_for_reading(const char *identifier);

  template <class InterfaceType>
    InterfaceType * open_for_writing(const char *identifier);


  static const unsigned int BBEL_FLAG_DATA;
  static const unsigned int BBEL_FLAG_READER;
  static const unsigned int BBEL_FLAG_WRITER;
  static const unsigned int BBEL_FLAG_INTERFACE;
  static const unsigned int BBEL_FLAG_ALL;
  void register_listener(BlackBoardEventListener *listener,
			 unsigned int flags = BBEL_FLAG_DATA);
  void unregister_listener(BlackBoardEventListener *listener);

  const BlackBoardMemoryManager *  memory_manager() const;

 private:
  Interface *  new_interface_instance(const char *type, const char *identifier);
  void         delete_interface_instance(Interface *interface);

  void *       find_interface_in_memory(const char *type, const char *identifier);
  unsigned int next_mem_serial();
  unsigned int next_instance_serial();
  void         create_interface(const char *type, const char *identifier,
				Interface* &interface, void* &ptr);

  char *       strip_class_type(const char *type);

  Interface *  writer_for_mem_serial(unsigned int mem_serial);

  void notify_of_interface_created(const char *type, const char *id) throw();
  void notify_of_writer_added(const char *uid) throw();
  void notify_of_writer_removed(const Interface *interface) throw();
  void notify_of_reader_added(const char *uid) throw();
  void notify_of_reader_removed(const Interface *interface) throw();

 private:
  bool                          bb_master;

  unsigned int                  instance_serial;

  BlackBoardMemoryManager      *memmgr;
  BlackBoardMessageManager     *msgmgr;
  Mutex                        *mutex;
  Module                       *iface_module;

  LockMap< unsigned int, Interface * >       writer_interfaces;
  LockMap< unsigned int, RefCountRWLock * >  rwlocks;

  typedef std::list< BlackBoardEventListener * >           BBelList;
  typedef LockHashMap< const char *, BBelList,
    __gnu_cxx::hash<const char *>, StringEquality>  BBelLockHashMap;

  typedef std::list< BlackBoardEventListener * >::iterator BBelListIterator;
  typedef BBelLockHashMap::iterator BBelLockHashMapIterator;

  BBelLockHashMap __bbel_interface;
  BBelLockHashMap __bbel_data;
  BBelLockHashMap __bbel_reader;
  BBelLockHashMap __bbel_writer;

};


/** Get interface of given type.
 * This will open a new interface for reading just like the non-template version of
 * open_for_reading(). But with the template method you will get a correctly typed object
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
BlackBoardInterfaceManager::open_for_reading(const char *identifier)
{
  char *type_name = strip_class_type(typeid(InterfaceType).name());
  InterfaceType *interface = dynamic_cast<InterfaceType *>(open_for_reading(type_name, identifier));
  delete[] type_name;
  if ( interface == 0 ) {
    throw TypeMismatchException("Interface (R) types do not match");
  } else {
    return interface;
  }
}


/** Open all interfaces of given type for reading.
 * This will create interface instances for all currently registered interfaces of
 * the given type. The result can be casted to the appropriate type.
 * @param id_prefix if set only interfaces whose ids have this prefix are returned
 * @return list of new fully initialized interface instances of requested type. The
 * is allocated using new and you have to free it using delete after you are done
 * with it!
 */
template <class InterfaceType>
std::list<InterfaceType *> *
BlackBoardInterfaceManager::open_all_of_type_for_reading(const char *id_prefix)
{
  char *type_name = strip_class_type(typeid(InterfaceType).name());
  std::list<Interface *> *il = open_all_of_type_for_reading(type_name, id_prefix);
  delete[] type_name;
  std::list<InterfaceType *> *rv = new std::list<InterfaceType *>();
  for (std::list<Interface *>::iterator i = il->begin(); i != il->end(); ++i) {
    InterfaceType *interface = dynamic_cast<InterfaceType *>(*i);
    if ( interface == 0 ) {
      // this really should never happen, but if it does we want to know,
      // because then we have a serious problem. We don't care about the
      // memleak here for that very reason.
      throw TypeMismatchException("open_all_off_type_for_reading: "
				  "Interface (R) types do not match");
    } else {
      rv->push_back(interface);
    }
  }

  delete il;

  return rv;
}


/** Get writer interface of given type.
 * This will open a new interface for writing just like the non-template version of
 * open_for_writing(). But with the template method you will get a correctly typed object
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
BlackBoardInterfaceManager::open_for_writing(const char *identifier)
{
  char *type_name = strip_class_type(typeid(InterfaceType).name());
  InterfaceType *interface;
  try {
    interface = dynamic_cast<InterfaceType *>(open_for_writing(type_name, identifier));
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
