 
/***************************************************************************
 *  interface_manager.cpp - BlackBoard interface manager
 *
 *  Generated: Mon Oct 09 19:08:29 2006
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

#include <blackboard/interface_manager.h>

#include <blackboard/bbconfig.h>
#include <blackboard/memory_manager.h>
#include <blackboard/message_manager.h>
#include <blackboard/exceptions.h>
#include <blackboard/interface_mem_header.h>

#include <interface/interface.h>

#include <core/threading/mutex.h>
#include <core/threading/refc_rwlock.h>
#include <core/exceptions/system.h>
#include <utils/system/dynamic_module/module_dl.h>

#include <stdlib.h>
#include <string>

using namespace std;

/** @class BlackBoardInterfaceManager blackboard/interface_manager.h
 * BlackBoard interface manager.
 * This class is used by the BlackBoard to manage interfaces stored in the
 * shared memory.
 *
 * An interface consists of a few blocks. First there is the storage block. This
 * is a chunk of memory in the shared memory segment where the actual data is stored.
 * Then there is the accessor object, an instance of a derivate of the Interface
 * class which is used to access the data in the shared memory segment.
 *
 * The interface manager keeps track of all the allocated interfaces. Events
 * can be triggered if a specific interface changes (like logging the data to
 * a file, sending it over the network or notifying another interface of such
 * a change).
 *
 * It also uses the memory manager to organize the chunks of memory that are used
 * for data storage.
 *
 * Interfaces can only be instantiated through the interface manager. The interface
 * manager instantiates an interface on request and guarantees that the instance
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
 * The interface manager can operate in two modes. In master mode the manager is
 * responsible for allocating and managing BlackBoard internal data. There must be
 * one and only one active master at any given time.
 * In slave mode the interface manager will contact the master to get serials etc.
 * for new interfaces. The slave mode is not yet fully supported.
 *
 * @see Interface
 * @see Message
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * The shared memory segment is created with data from bbconfig.h.
 * @param bb_master set to true, if this interface manager should be the master.
 * @see bbconfig.h
 */
BlackBoardInterfaceManager::BlackBoardInterfaceManager(bool bb_master)
{
  this->bb_master = bb_master;

  memmgr = new BlackBoardMemoryManager(BLACKBOARD_MEMORY_SIZE,
				       BLACKBOARD_VERSION,
				       bb_master,
				       BLACKBOARD_MAGIC_TOKEN);

  instance_serial = 1;
  mutex = new Mutex();
  try {
    iface_module = new ModuleDL( LIBDIR"/libinterfaces.so" );
    iface_module->open();
  } catch (Exception &e) {
    e.append("BlackBoardInterfaceManager cannot open interface module");
    delete mutex;
    delete iface_module;
    throw;
  }

  msgmgr = new BlackBoardMessageManager(this);

  writer_interfaces.clear();
  rwlocks.clear();
}


/** Destructor */
BlackBoardInterfaceManager::~BlackBoardInterfaceManager()
{
  delete mutex;
  delete iface_module;
  delete memmgr;
  delete msgmgr;
}


/** Strip numbers at the beginning of the class type.
 * This has been implemented by observations of C++ class names as returned by GCC's
 * typeid operator.
 * @param type type name to strip
 * @return stripped class type
 */
char *
BlackBoardInterfaceManager::strip_class_type(const char *type)
{
  string t = type;
  t = t.substr( t.find_first_not_of("0123456789") );
  char *rv = new char[t.length() + 1];
  strcpy(rv, t.c_str());
  return rv;
}


/** Creates a new interface instance.
 * This method will look in the libinterfaces shared object for a factory function
 * for the interface of the given type. If this was found a new instance of the
 * interface is returned.
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @return a new instance of the requested interface type
 * @exception BlackBoardInterfaceNotFoundException thrown if the factory function
 * for the given interface type could not be found
 */
Interface *
BlackBoardInterfaceManager::new_interface_instance(const char *type, const char *identifier)
{
  char *generator_name = (char *)malloc(strlen("new") + strlen(type) + 1);
  sprintf(generator_name, "new%s", type);
  if ( ! iface_module->hasSymbol(generator_name) ) {
    free(generator_name);
    throw BlackBoardInterfaceNotFoundException(type);
  }

  InterfaceFactoryFunc iff = (InterfaceFactoryFunc)iface_module->getSymbol(generator_name);

  Interface *iface = iff();

  iface->__instance_serial = next_instance_serial();
  strncpy(iface->__type, type, __INTERFACE_TYPE_SIZE);
  strncpy(iface->__id, identifier, __INTERFACE_ID_SIZE);
  iface->__interface_mediator = this;
  iface->__message_mediator   = msgmgr;

  free(generator_name);
  return iface;
}


/** Destroy an interface instance.
 * The destroyer function for the given interface is called to destroy the given
 * interface instance.
 * @param interface to destroy
 * @exception BlackBoardInterfaceNotFoundException thrown if the destroyer function
 * for the given interface could not be found. The interface will not be freed.
 */
void
BlackBoardInterfaceManager::delete_interface_instance(Interface *interface)
{
  char *destroyer_name = (char *)malloc(strlen("delete") + strlen(interface->__type) + 1);
  sprintf(destroyer_name, "delete%s", interface->__type);
  if ( ! iface_module->hasSymbol(destroyer_name) ) {
    free(destroyer_name);
    throw BlackBoardInterfaceNotFoundException(interface->__type);
  }

  InterfaceDestroyFunc idf = (InterfaceDestroyFunc)iface_module->getSymbol(destroyer_name);
  idf(interface);
  free(destroyer_name);
}


/** search memory chunks if the desired interface has been allocated already.
 * @param type type of the interface to look for
 * @param identifier identifier of the interface to look for
 * @return a pointer to the memory of the interface or NULL if not found
 */
void *
BlackBoardInterfaceManager::find_interface_in_memory(const char *type, const char *identifier)
{
  interface_header_t *ih;
  BlackBoardMemoryManager::ChunkIterator cit;
  for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
    ih = (interface_header_t *)*cit;
    if ( (strncmp(ih->type, type, __INTERFACE_TYPE_SIZE) == 0) &&
	 (strncmp(ih->id, identifier, __INTERFACE_ID_SIZE) == 0 )
	 ) {
      // found it!
      return *cit;
    }
  }

  return NULL;
}


/** Get next mem serial.
 * @return next unique memory serial
 */
unsigned int
BlackBoardInterfaceManager::next_mem_serial()
{
  unsigned int serial = 1;
  interface_header_t *ih;
  BlackBoardMemoryManager::ChunkIterator cit;
  for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
    ih = (interface_header_t *)*cit;
    if ( ih->serial >= serial ) {
      serial = ih->serial + 1;
    }
  }

  return serial;
}


/** Get next instance serial.
 * @return next unique instance serial
 */
unsigned int
BlackBoardInterfaceManager::next_instance_serial()
{
  if ( bb_master ) {
    // simple, just increment value and return it
    return instance_serial++;
  } else {
    throw BBNotMasterException("Instance serial can only be requested by BB Master");
  }
}


/** Create an interface instance.
 * This will create a new interface instance. Storage in the shared memory
 * is allocated to hold the interface data.
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @param interface reference to a pointer where the interface will be created
 * @param ptr reference to pointer of interface memory
 * @exception OutOfMemoryException thrown if there is not enough memory in the
 * BlackBoard to create the interface
 */
void
BlackBoardInterfaceManager::create_interface(const char *type, const char *identifier,
					     Interface* &interface, void* &ptr)
{
  interface_header_t *ih;

  // create new interface and allocate appropriate chunk
  interface = new_interface_instance(type, identifier);
  try {
    ptr = memmgr->alloc_nolock(interface->datasize() + sizeof(interface_header_t));
    ih  = (interface_header_t *)ptr;
  } catch (OutOfMemoryException &e) {
    e.append("BlackBoardInterfaceManager::createInterface: interface of type %s could not be created", type);
    memmgr->unlock();
    mutex->unlock();
    throw;
  }
  memset(ptr, 0, interface->datasize() + sizeof(interface_header_t));

  strncpy(ih->type, type, __INTERFACE_TYPE_SIZE);
  strncpy(ih->id, identifier, __INTERFACE_ID_SIZE);

  ih->refcount           = 0;
  ih->serial             = next_mem_serial();
  ih->flag_writer_active = 0;
  ih->num_readers        = 0;
  rwlocks[ih->serial] = new RefCountRWLock();

  interface->__mem_real_ptr  = ptr;
  interface->__mem_data_ptr  = (char *)ptr + sizeof(interface_header_t);

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
BlackBoardInterfaceManager::open_for_reading(const char *type, const char *identifier)
{
  mutex->lock();
  Interface *iface = NULL;
  void *ptr = NULL;
  interface_header_t *ih;

  memmgr->lock();

  ptr = find_interface_in_memory(type, identifier);

  if ( ptr != NULL ) {
    // found, instantiate new interface for given memory chunk
    iface = new_interface_instance(type, identifier);
    iface->__mem_real_ptr = ptr;
    iface->__mem_data_ptr = (char *)ptr + sizeof(interface_header_t);
    ih  = (interface_header_t *)ptr;
    rwlocks[ih->serial]->ref();
  } else {
    create_interface(type, identifier, iface, ptr);
    ih  = (interface_header_t *)ptr;
  }

  iface->__write_access = false;
  iface->__rwlock = rwlocks[ih->serial];
  iface->__mem_serial = ih->serial;
  iface->__message_queue = new MessageQueue(iface->__mem_serial, iface->__instance_serial);
  ih->refcount++;
  ih->num_readers++;

  memmgr->unlock();
  mutex->unlock();
  return iface;
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
BlackBoardInterfaceManager::open_all_of_type_for_reading(const char *type, const char *id_prefix)
{
  mutex->lock();
  memmgr->lock();

  bool match = false;
  std::list<Interface *> *rv = new std::list<Interface *>();

  Interface *iface = NULL;
  interface_header_t *ih;
  BlackBoardMemoryManager::ChunkIterator cit;
  for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
    ih = (interface_header_t *)*cit;

    if (NULL == id_prefix) {
      if (strncmp(ih->type, type, __INTERFACE_TYPE_SIZE) == 0) {
	match = true;
      }
    } else {
      if (strncmp(ih->type, type, __INTERFACE_TYPE_SIZE) == 0 &&
	  strncmp(id_prefix, ih->id, strlen(id_prefix)) == 0) {
	match = true;
      }
    }
      
    if (match) {
      // found one!
      // open 
      void *ptr = *cit;
      iface = new_interface_instance(ih->type, ih->id);
      iface->__mem_real_ptr = ptr;
      iface->__mem_data_ptr = (char *)ptr + sizeof(interface_header_t);
      ih  = (interface_header_t *)ptr;
      rwlocks[ih->serial]->ref();

      iface->__write_access = false;
      iface->__rwlock = rwlocks[ih->serial];
      iface->__mem_serial = ih->serial;
      iface->__message_queue = new MessageQueue(iface->__mem_serial, iface->__instance_serial);
      ih->refcount++;
      ih->num_readers++;

      rv->push_back(iface);
    }
  }

  mutex->unlock();
  memmgr->unlock();

  return rv;
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
BlackBoardInterfaceManager::open_for_writing(const char *type, const char *identifier)
{
  mutex->lock();
  memmgr->lock();

  Interface *iface = NULL;
  void *ptr = NULL;
  interface_header_t *ih;

  ptr = find_interface_in_memory(type, identifier);

  if ( ptr != NULL ) {
    // found, check if there is already a writer
    //instantiate new interface for given memory chunk
    ih  = (interface_header_t *)ptr;
    if ( ih->flag_writer_active ) {
      memmgr->unlock();
      mutex->unlock();
      throw BlackBoardWriterActiveException(identifier, type);
    }
    iface = new_interface_instance(type, identifier);
    iface->__mem_real_ptr = ptr;
    iface->__mem_data_ptr = (char *)ptr + sizeof(interface_header_t);
    rwlocks[ih->serial]->ref();
  } else {
    create_interface(type, identifier, iface, ptr);
    ih = (interface_header_t *)ptr;
  }

  iface->__write_access = true;
  iface->__rwlock  = rwlocks[ih->serial];
  iface->__mem_serial = ih->serial;
  iface->__message_queue = new MessageQueue(iface->__mem_serial, iface->__instance_serial);
  ih->flag_writer_active = 1;
  ih->refcount++;

  memmgr->unlock();
  mutex->unlock();

  writer_interfaces[iface->__mem_serial] = iface;

  return iface;
}


/** Close interface.
 * @param interface interface to close
 */
void
BlackBoardInterfaceManager::close(Interface *interface)
{
  mutex->lock();

  // reduce refcount and free memory if refcount is zero
  interface_header_t *ih = (interface_header_t *)interface->__mem_real_ptr;
  if ( --(ih->refcount) == 0 ) {
    // redeem from memory
    memmgr->free( interface->__mem_real_ptr );
  } else {
    if ( interface->__write_access ) {
      ih->flag_writer_active = 0;
      writer_interfaces.erase( interface->__mem_serial );
    } else {
      ih->num_readers--;
    }
  }

  delete_interface_instance( interface );

  mutex->unlock();
}


/** Get the writer interface for the given mem serial.
 * @param mem_serial memory serial to get writer for
 * @return writer interface for given mem serial, or NULL if non exists
 * @exception BlackBoardNoWritingInstanceException thrown if no writer
 * was found for the given interface.
 */
Interface *
BlackBoardInterfaceManager::writer_for_mem_serial(unsigned int mem_serial)
{
  if ( writer_interfaces.find(mem_serial) != writer_interfaces.end() ) {
    return writer_interfaces[mem_serial];
  } else {
    throw BlackBoardNoWritingInstanceException();
  }
}


/** Get memory manager.
 * This returns a pointer to the used memory manager. The return type
 * is declared const. Use this only for debugging purposes to output info about
 * the BlackBoard memory.
 * @return const pointer to memory manager
 */
const BlackBoardMemoryManager *
BlackBoardInterfaceManager::memory_manager() const
{
  return memmgr;
}


/** Check if a writer exists.
 * Check if there is any writer for the given interface.
 * @param interface interface to check
 * @return true, if there is any writer for the given interface, false otherwise
 */
bool
BlackBoardInterfaceManager::exists_writer(const Interface *interface) const
{
  return (writer_interfaces.find(interface->__mem_serial) != writer_interfaces.end());
}


/** Notify of data change.
 * Notify all subscribers of the given interface of a data change.
 * This also influences logging and sending data over the network so it is
 * mandatory to call this function! The interface base class write method does
 * that for you.
 * @param interface interface whose subscribers to notify
 * @see Interface::write()
 */
void
BlackBoardInterfaceManager::notify_of_data_change(const Interface *interface)
{
  // do nothing for now
  // printf("Interface %s of type %s has changed\n", interface->id(), interface->type());
}
