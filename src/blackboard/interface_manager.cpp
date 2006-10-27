 
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <blackboard/interface_manager.h>

#include <blackboard/memory_manager.h>
#include <blackboard/exceptions.h>
#include <blackboard/interface_mem_header.h>

#include <interfaces/interface.h>
#include <interfaces/blackboard.h>

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
 * responsible for allocating and managing BlackBoard internal data (stored in the
 * BlackBoardInternalsInterface with identifier "FawkesBlackBoard"). There must be
 * one and only one active master at any given time.
 * In slave mode the interface manager will contact the master to get serials etc.
 * for new interfaces.
 *
 * @see Interface
 * @see Message
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param memmgr Current memory manager that shall be used to allocate memory for
 * interface data storage
 * @param bb_master set to true, if this interface manager should be the master.
 */
BlackBoardInterfaceManager::BlackBoardInterfaceManager(BlackBoardMemoryManager *memmgr,
						       bool bb_master)
{
  this->bb_master = bb_master;
  this->memmgr = memmgr;
  mutex = new Mutex();
  iface_module = new ModuleDL( LIBDIR"/libinterfaces.so" );
  if ( ! iface_module->open() ) {
    throw BlackBoardCannotFindInterfaceModuleException();
  }
  if ( bb_master ) {
    // may throw exception! This is ok and correct behavior
    Interface *iface = openForWriting("BlackBoardInternalsInterface", "FawkesBlackBoard");
    internals = dynamic_cast<BlackBoardInternalsInterface *>(iface);
    internals->setInstanceSerial( internals->serial() );
    internals->write();
  } else {
    openInternalsNonMaster();
  }
}


/** Destructor */
BlackBoardInterfaceManager::~BlackBoardInterfaceManager()
{
  close(internals);
  delete mutex;
  delete iface_module;
}


/** Strip numbers at the beginning of the class type.
 * This has been implemented by observations of C++ class names as returned by GCC's
 * typeid operator.
 * @param type type name to strip
 * @return stripped class type
 */
char *
BlackBoardInterfaceManager::stripClassType(const char *type)
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
BlackBoardInterfaceManager::newInterfaceInstance(const char *type, const char *identifier)
{
  char *generator_name = (char *)malloc(strlen("new") + strlen(type) + 1);
  sprintf(generator_name, "new%s", type);
  if ( ! iface_module->hasSymbol(generator_name) ) {
    free(generator_name);
    throw BlackBoardInterfaceNotFoundException(type);
  }

  InterfaceFactoryFunc iff = (InterfaceFactoryFunc)iface_module->getSymbol(generator_name);

  Interface *iface = iff();

  iface->instance_serial = getNextInstanceSerial();
  strncpy(iface->_type, type, __INTERFACE_TYPE_SIZE);
  strncpy(iface->_id, identifier, __INTERFACE_ID_SIZE);

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
BlackBoardInterfaceManager::deleteInterfaceInstance(Interface *interface)
{
  char *destroyer_name = (char *)malloc(strlen("delete") + strlen(interface->_type) + 1);
  sprintf(destroyer_name, "delete%s", interface->_type);
  if ( ! iface_module->hasSymbol(destroyer_name) ) {
    free(destroyer_name);
    throw BlackBoardInterfaceNotFoundException(interface->_type);
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
BlackBoardInterfaceManager::findInterfaceInMemory(const char *type, const char *identifier)
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
BlackBoardInterfaceManager::getNextMemSerial()
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
BlackBoardInterfaceManager::getNextInstanceSerial()
{
  unsigned int serial = 0;
  if ( bb_master ) {
    // simple, just increment value and return it
    if ( internals != NULL ) {
      serial = internals->getInstanceSerial() + 1;
      internals->setInstanceSerial( serial );
      internals->write();
    } else {
      // this is the serial for the internals interface instance, assumably!
      return 1;
    }
  } else {
    // Send message to request serial
    /*
    BlackBoardIntenrnalsInterface::GetInstanceSerialMessage *m;
    m = new BlackBoardIntenrnalsInterface::GetInstanceSerialMessage();
    Message reply = internals->sendWithReplyAndBlock(m);
    BlackBoardIntenrnalsInterface::GetInstanceSerialMessageReply *r;
    r = dynamic_cast<BlackBoardIntenrnalsInterface::GetInstanceSerialMessageReply *>(reply);
    if ( r != NULL ) {
      serial = r->getInstanceSerial();
      delete r;
    }
    */
  }
  return serial;
}


/** Open internals interface as slave.
 * @return pointer to a new instance of BlackBoardInternalsInterface
 * @exception BlackBoardNoMasterAliveException thrown if there is no master alive
 * for this BlackBoard. Some weird error occured, investigate!
 */
BlackBoardInternalsInterface *
BlackBoardInterfaceManager::openInternalsNonMaster()
{
  Interface *iface = NULL;

  mutex->lock();
  memmgr->lock();
  void *ptr = findInterfaceInMemory("BlackBoardInternalsInterface", "FawkesBlackBoard");

  if ( ptr ) {
    // good, check if there is a writer
    interface_header_t *ih = (interface_header_t *)ptr;
    if ( ! ih->flag_writer_active ) {
      throw BlackBoardNoMasterAliveException();
    }

    iface = newInterfaceInstance("BlackBoardInternalsInterface", "FawkesBlackBoard");
    iface->mem_real_ptr = ptr;
    iface->mem_data_ptr = (char *)ptr + sizeof(interface_header_t);

    iface->interface_mediator = this;
    iface->write_access = false;
    ih->rwlock->ref();
    iface->rwlock = ih->rwlock;
    iface->mem_serial = ih->serial;
    ih->refcount++;

  } else {
    throw BlackBoardNoMasterAliveException();
  }

  memmgr->unlock();
  mutex->unlock();

  return dynamic_cast<BlackBoardInternalsInterface *>(iface);
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
BlackBoardInterfaceManager::createInterface(const char *type, const char *identifier,
					    Interface* &interface, void* &ptr)
{
  interface_header_t *ih;

  // create new interface and allocate appropriate chunk
  interface = newInterfaceInstance(type, identifier);
  try {
    ptr = memmgr->alloc_nolock(interface->datasize() + sizeof(interface_header_t));
    ih  = (interface_header_t *)ptr;
  } catch (OutOfMemoryException &e) {
    e.append("BlackBoardInterfaceManager::createInterface: interface of type %s could not be created", type);
    memmgr->unlock();
    mutex->unlock();
    throw;
  }
  strncpy(ih->type, type, __INTERFACE_TYPE_SIZE);
  strncpy(ih->id, identifier, __INTERFACE_ID_SIZE);

  ih->refcount     = 0;
  ih->rwlock       = new RefCountRWLock();
  ih->serial       = getNextMemSerial();

  interface->mem_real_ptr  = ptr;
  interface->mem_data_ptr  = (char *)ptr + sizeof(interface_header_t);

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
BlackBoardInterfaceManager::openForReading(const char *type, const char *identifier)
{
  mutex->lock();
  Interface *iface = NULL;
  void *ptr = NULL;
  interface_header_t *ih;

  memmgr->lock();

  ptr = findInterfaceInMemory(type, identifier);

  if ( ptr != NULL ) {
    // found, instantiate new interface for given memory chunk
    iface = newInterfaceInstance(type, identifier);
    iface->mem_real_ptr = ptr;
    iface->mem_data_ptr = (char *)ptr + sizeof(interface_header_t);
    ih  = (interface_header_t *)ptr;
    ih->rwlock->ref();
  } else {
    createInterface(type, identifier, iface, ptr);
    ih = (interface_header_t *)ptr;
    ih->serial = getNextMemSerial();
  }

  iface->interface_mediator = this;
  iface->write_access = false;
  iface->rwlock = ih->rwlock;
  iface->mem_serial = ih->serial;
  ih->refcount++;

  memmgr->unlock();
  mutex->unlock();
  return iface;
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
BlackBoardInterfaceManager::openForWriting(const char *type, const char *identifier)
{
  mutex->lock();
  memmgr->lock();

  Interface *iface = NULL;
  void *ptr = NULL;
  interface_header_t *ih;

  ptr = findInterfaceInMemory(type, identifier);

  if ( ptr != NULL ) {
    // found, check if there is already a writer
    //instantiate new interface for given memory chunk
    ih  = (interface_header_t *)ptr;
    if ( ih->flag_writer_active ) {
      memmgr->unlock();
      mutex->unlock();
      throw BlackBoardWriterActiveException(identifier, type);
    }
    iface = newInterfaceInstance(type, identifier);
    iface->mem_real_ptr = ptr;
    iface->mem_data_ptr = (char *)ptr + sizeof(interface_header_t);
    ih->rwlock->ref();
  } else {
    createInterface(type, identifier, iface, ptr);
    ih = (interface_header_t *)ptr;
  }

  iface->interface_mediator = this;
  iface->write_access = true;
  iface->rwlock  = ih->rwlock;
  iface->mem_serial = ih->serial;
  ih->flag_writer_active = 1;
  ih->refcount++;

  memmgr->unlock();
  mutex->unlock();
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
  interface_header_t *ih = (interface_header_t *)interface->mem_real_ptr;
  if ( --(ih->refcount) == 0 ) {
    // redeem from memory
    memmgr->free( interface->mem_real_ptr );
  } else {
    if ( interface->write_access ) {
      ih->flag_writer_active = 0;
    }
  }

  deleteInterfaceInstance( interface );

  mutex->unlock();
}


/** Check if a writer exists.
 * Check if there is any writer for the given interface.
 * @param interface interface to check
 * @return true, if there is any writer for the given interface, false otherwise
 */
bool
BlackBoardInterfaceManager::existsWriter(const Interface *interface) const
{
  // search memory chunks if the desired interface has been allocated already
  BlackBoardMemoryManager::ChunkIterator cit;
  for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
    if ( interface->mem_real_ptr == *cit ) {
      interface_header_t *ih = (interface_header_t *)interface->mem_real_ptr;
      return ih->flag_writer_active;
    }
  }

  return false;
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
BlackBoardInterfaceManager::notifyOfDataChange(const Interface *interface)
{
  // do nothing for now
  // printf("Interface %s of type %s has changed\n", interface->id(), interface->type());
}
