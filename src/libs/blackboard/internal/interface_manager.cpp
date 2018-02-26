 
/***************************************************************************
 *  interface_manager.cpp - BlackBoard interface manager
 *
 *  Created: Mon Oct 09 19:08:29 2006
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

#include <blackboard/internal/interface_manager.h>

#include <blackboard/blackboard.h>
#include <blackboard/internal/memory_manager.h>
#include <blackboard/internal/message_manager.h>
#include <blackboard/exceptions.h>
#include <blackboard/internal/interface_mem_header.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <blackboard/internal/instance_factory.h>
#include <blackboard/internal/notifier.h>

#include <interface/interface.h>
#include <interface/interface_info.h>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/refc_rwlock.h>
#include <core/exceptions/system.h>
#include <utils/system/dynamic_module/module.h>
#include <utils/time/time.h>

#include <cstdlib>
#include <cstring>
#include <fnmatch.h>

namespace fawkes {

/** @class BlackBoardInterfaceManager <blackboard/internal/interface_manager.h>
 * BlackBoard interface manager.
 * This class is used by the BlackBoard to manage interfaces stored in the
 * shared memory.
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * The shared memory segment is created with data from bbconfig.h.
 * @param bb_memmgr BlackBoard memory manager to use
 * @param bb_msgmgr BlackBoard message manager to use
 * @param bb_notifier BlackBoard notifier to all for events
 * @see bbconfig.h
 */
BlackBoardInterfaceManager::BlackBoardInterfaceManager(BlackBoardMemoryManager *bb_memmgr,
						       BlackBoardMessageManager *bb_msgmgr,
						       BlackBoardNotifier *bb_notifier)
{
  memmgr = bb_memmgr;
  msgmgr = bb_msgmgr;
  notifier = bb_notifier;

  instance_serial = 1;
  instance_factory = new BlackBoardInstanceFactory();
  mutex = new Mutex();

  writer_interfaces.clear();
  rwlocks.clear();
}


/** Destructor */
BlackBoardInterfaceManager::~BlackBoardInterfaceManager()
{
  delete mutex;
  delete instance_factory;
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
BlackBoardInterfaceManager::new_interface_instance(const char *type, const char *identifier, const char *owner)
{
  Interface *iface = instance_factory->new_interface_instance(type, identifier);

  iface->set_instance_serial(next_instance_serial());
  iface->set_mediators(this, msgmgr);
  if (owner) iface->set_owner(owner);
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
  if (owner_info_.find(interface->uid()) != owner_info_.end()) {
    OwnerInfo &info = owner_info_[interface->uid()];
    if (interface->is_writer()) {
      if (info.writer == interface) {
	info.writer = NULL;
      }
    } else {
      info.readers.remove(interface);
    }
  }
  instance_factory->delete_interface_instance(interface);
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
	 (strncmp(ih->id, identifier, __INTERFACE_ID_SIZE) == 0)
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
  if ( memmgr->is_master() ) {
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
					     const char *owner,
					     Interface* &interface, void* &ptr)
{
  interface_header_t *ih;

  // create new interface and allocate appropriate chunk
  interface = new_interface_instance(type, identifier, owner);
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

  strncpy(ih->type, type, __INTERFACE_TYPE_SIZE-1);
  strncpy(ih->id, identifier, __INTERFACE_ID_SIZE-1);
  memcpy(ih->hash, interface->hash(), __INTERFACE_HASH_SIZE);

  ih->refcount           = 0;
  ih->serial             = next_mem_serial();
  ih->flag_writer_active = 0;
  ih->num_readers        = 0;
  rwlocks[ih->serial] = new RefCountRWLock();

  interface->set_memory(ih->serial, ptr, (char *)ptr + sizeof(interface_header_t));
}


/** Open interface for reading.
 * This will create a new interface instance of the given type. The result can be
 * casted to the appropriate type.
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @param owner name of entity which opened this interface. If using the BlackBoardAspect
 * to access the blackboard leave this untouched unless you have a good reason.
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 */
Interface *
BlackBoardInterfaceManager::open_for_reading(const char *type, const char *identifier, const char *owner)
{
  if (strlen(type) > __INTERFACE_TYPE_SIZE) {
    throw Exception("Interface type '%s' too long, maximum length is %zu",
		    type, __INTERFACE_TYPE_SIZE);
  }
  if (strlen(identifier) > __INTERFACE_ID_SIZE) {
    throw Exception("Interface ID '%s' too long, maximum length is %zu",
		    type, __INTERFACE_ID_SIZE);
  }

  mutex->lock();
  Interface *iface = NULL;
  void *ptr = NULL;
  interface_header_t *ih;
  bool created = false;

  memmgr->lock();

  ptr = find_interface_in_memory(type, identifier);

  try {
    if ( ptr != NULL ) {
      // found, instantiate new interface for given memory chunk
      iface = new_interface_instance(type, identifier, owner);
      ih  = (interface_header_t *)ptr;
      if ( (iface->hash_size() != __INTERFACE_HASH_SIZE ) ||
	   (memcmp(iface->hash(), ih->hash, __INTERFACE_HASH_SIZE) != 0) ) {
	throw BlackBoardInterfaceVersionMismatchException();
      }
      iface->set_memory(ih->serial, ptr, (char *)ptr + sizeof(interface_header_t));
      rwlocks[ih->serial]->ref();
    } else {
      created = true;
      create_interface(type, identifier, owner, iface, ptr);
      ih  = (interface_header_t *)ptr;
    }

    owner_info_[iface->uid()].readers.push_back(iface);
    iface->set_readwrite(false, rwlocks[ih->serial]);
    ih->refcount++;
    ih->num_readers++;

    memmgr->unlock();
    mutex->unlock();

    if ( created ) {
      notifier->notify_of_interface_created(type, identifier);
    }
    notifier->notify_of_reader_added(iface, iface->serial());

  } catch (Exception &e) {
    if (iface)  delete_interface_instance(iface);
    memmgr->unlock();
    mutex->unlock();
    throw;
  }

  return iface;
}


/** Open all interfaces of the given type for reading.
 * This will create interface instances for all currently registered interfaces of
 * the given type. The result can be casted to the appropriate type.
 * @param type_pattern pattern of interface types to open, supports wildcards
 * similar to filenames (*, ?, []), see "man fnmatch" for all supported.
 * @param id_pattern pattern of interface IDs to open, supports wildcards similar
 * to filenames (*, ?, []), see "man fnmatch" for all supported.
 * @param owner name of entity which opened this interface. If using the BlackBoardAspect
 * to access the blackboard leave this untouched unless you have a good reason.
 * @return list of new fully initialized interface instances of requested type. The
 * is allocated using new and you have to free it using delete after you are done
 * with it!
 */
std::list<Interface *>
BlackBoardInterfaceManager::open_multiple_for_reading(const char *type_pattern,
						      const char *id_pattern,
						      const char *owner)
{
  mutex->lock();
  memmgr->lock();

  std::list<Interface *> rv;

  Interface *iface = NULL;
  interface_header_t *ih;
  BlackBoardMemoryManager::ChunkIterator cit;

  try {
    for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
      iface = NULL;
      ih = (interface_header_t *)*cit;

      // ensure 0-termination
      char type[__INTERFACE_TYPE_SIZE + 1];
      char id[__INTERFACE_ID_SIZE + 1];
      type[__INTERFACE_TYPE_SIZE] = 0;
      id[__INTERFACE_TYPE_SIZE] = 0;
      strncpy(type, ih->type, __INTERFACE_TYPE_SIZE);
      strncpy(id, ih->id, __INTERFACE_ID_SIZE);

      if ((fnmatch(type_pattern, type, 0) == FNM_NOMATCH) ||
	  (fnmatch(id_pattern, id, 0) == FNM_NOMATCH) ) {
	// type or ID prefix does not match, go on
	continue;
      }

      void *ptr = *cit;
      iface = new_interface_instance(ih->type, ih->id, owner);
      iface->set_memory(ih->serial, ptr, (char *)ptr + sizeof(interface_header_t));

      if ( (iface->hash_size() != __INTERFACE_HASH_SIZE ) ||
	   (memcmp(iface->hash(), ih->hash, __INTERFACE_HASH_SIZE) != 0) ) {
	throw BlackBoardInterfaceVersionMismatchException();
      }

      rwlocks[ih->serial]->ref();

      owner_info_[iface->uid()].readers.push_back(iface);
      iface->set_readwrite(false, rwlocks[ih->serial]);
      ih->refcount++;
      ih->num_readers++;

      rv.push_back(iface);
    }

    mutex->unlock();
    memmgr->unlock();

    for (std::list<Interface *>::iterator j = rv.begin(); j != rv.end(); ++j) {
      notifier->notify_of_reader_added(*j, (*j)->serial());
    }


  } catch (Exception &e) {
    if (iface)  delete_interface_instance( iface );
    for (std::list<Interface *>::iterator i = rv.begin(); i != rv.end(); ++i) {
      delete_interface_instance(*i);
    }
    memmgr->unlock();
    mutex->unlock();
    throw;
  }

  return rv;
}


/** Open interface for writing.
 * This will create a new interface instance of the given type. The result can be
 * casted to the appropriate type. This will only succeed if there is not already
 * a writer for the given interface type/id!
 * @param type type of the interface
 * @param identifier identifier of the interface
 * @param owner name of entity which opened this interface. If using the BlackBoardAspect
 * to access the blackboard leave this untouched unless you have a good reason.
 * @return new fully initialized interface instance of requested type
 * @exception OutOfMemoryException thrown if there is not enough free space for
 * the requested interface.
 * @exception BlackBoardWriterActiveException thrown if there is already a writing
 * instance with the same type/id
 */
Interface *
BlackBoardInterfaceManager::open_for_writing(const char *type, const char *identifier, const char *owner)
{
  if (strlen(type) > __INTERFACE_TYPE_SIZE) {
    throw Exception("Interface type '%s' too long, maximum length is %zu",
		    type, __INTERFACE_TYPE_SIZE);
  }
  if (strlen(identifier) > __INTERFACE_ID_SIZE) {
    throw Exception("Interface ID '%s' too long, maximum length is %zu",
		    type, __INTERFACE_ID_SIZE);
  }

  mutex->lock();
  memmgr->lock();

  Interface *iface = NULL;
  void *ptr = NULL;
  interface_header_t *ih;
  bool created = false;

  try {
    ptr = find_interface_in_memory(type, identifier);

    if ( ptr != NULL ) {
      // found, check if there is already a writer
      //instantiate new interface for given memory chunk
      ih  = (interface_header_t *)ptr;
      if ( ih->flag_writer_active ) {
	throw BlackBoardWriterActiveException(identifier, type);
      }
      iface = new_interface_instance(type, identifier, owner);
      if ( (iface->hash_size() != __INTERFACE_HASH_SIZE ) ||
	   (memcmp(iface->hash(), ih->hash, __INTERFACE_HASH_SIZE) != 0) ) {
	throw BlackBoardInterfaceVersionMismatchException();
      }
      iface->set_memory(ih->serial, ptr, (char *)ptr + sizeof(interface_header_t));
      rwlocks[ih->serial]->ref();
    } else {
      created = true;
      create_interface(type, identifier, owner, iface, ptr);
      ih = (interface_header_t *)ptr;
    }

    owner_info_[iface->uid()].writer = iface;
    iface->set_readwrite(true, rwlocks[ih->serial]);
    ih->flag_writer_active = 1;
    ih->refcount++;

    memmgr->unlock();
    writer_interfaces[ih->serial] = iface;

    mutex->unlock();

    if ( created ) {
      notifier->notify_of_interface_created(type, identifier);
    }
    notifier->notify_of_writer_added(iface, iface->serial());
  } catch (Exception &e) {
    if (iface)  delete_interface_instance(iface);
    memmgr->unlock();
    mutex->unlock();
    throw;
  }

  return iface;
}


/** Close interface.
 * @param interface interface to close
 */
void
BlackBoardInterfaceManager::close(Interface *interface)
{
  if ( interface == NULL ) return;
  mutex->lock();
  bool destroyed = false;

  // reduce refcount and free memory if refcount is zero
  interface_header_t *ih = (interface_header_t *)interface->__mem_real_ptr;
  bool killed_writer = interface->__write_access;
  if ( --(ih->refcount) == 0 ) {
    // redeem from memory
    if ( interface->__write_access ) {
      writer_interfaces.erase( interface->__mem_serial );
    }
    memmgr->free( interface->__mem_real_ptr );
    destroyed = true;
  } else {
    if ( interface->__write_access ) {
      ih->flag_writer_active = 0;
      writer_interfaces.erase( interface->__mem_serial );
    } else {
      ih->num_readers--;
    }
  }

  mutex->unlock();
  if (killed_writer) {
    notifier->notify_of_writer_removed(interface, interface->serial());
  } else {
    notifier->notify_of_reader_removed(interface, interface->serial());
  }
  if ( destroyed ) {
    notifier->notify_of_interface_destroyed(interface->__type, interface->__id);
  }

  MutexLocker lock(mutex);
  delete_interface_instance( interface );
}


/** Get a list of interfaces.
 * @return list of currently existing interfaces. List may be outdated on
 * return since there maybe concurrent actions.
 */
InterfaceInfoList *
BlackBoardInterfaceManager::list_all() const
{
  InterfaceInfoList *infl = new InterfaceInfoList();

  memmgr->lock();
  interface_header_t *ih;
  BlackBoardMemoryManager::ChunkIterator cit;
  for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
    ih = (interface_header_t *)*cit;
    Interface::interface_data_ts_t *data_ts =
      (Interface::interface_data_ts_t *)((char *)*cit + sizeof(interface_header_t));
    char type[__INTERFACE_TYPE_SIZE + 1];
    char id[__INTERFACE_ID_SIZE + 1];
    // ensure NULL-termination
    type[__INTERFACE_TYPE_SIZE] = 0;
    id[__INTERFACE_ID_SIZE] = 0;
    strncpy(type, ih->type, __INTERFACE_TYPE_SIZE);
    strncpy(id, ih->id, __INTERFACE_ID_SIZE);
    std::string uid = std::string(type) + "::" + id;
    infl->append(ih->type, ih->id, ih->hash, ih->serial,
		 ih->flag_writer_active, ih->num_readers,
		 readers(uid), writer(uid),
		 Time(data_ts->timestamp_sec, data_ts->timestamp_usec));
  }

  memmgr->unlock();

  return infl;
}


/** Get a constrained list of interfaces.
 * @param type_pattern tyoe pattern, may contain shell-like wildcards * (any number
 * of characters) and ? (one character), cf. man fnmatch().
 * @param id_pattern ID pattern, may contain shell-like wildcards * (any number
 * of characters) and ? (one character), cf. man fnmatch().
 * @return list of currently existing interfaces matching the given type and
 * ID patterns. List may be outdated on return since there maybe concurrent
 * actions.
 */
InterfaceInfoList *
BlackBoardInterfaceManager::list(const char *type_pattern,
				 const char *id_pattern) const
{
  InterfaceInfoList *infl = new InterfaceInfoList();

  memmgr->lock();
  interface_header_t *ih;
  BlackBoardMemoryManager::ChunkIterator cit;
  for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
    ih = (interface_header_t *)*cit;
    Interface::interface_data_ts_t *data_ts =
      (Interface::interface_data_ts_t *)((char *)*cit + sizeof(interface_header_t));
    char type[__INTERFACE_TYPE_SIZE + 1];
    char id[__INTERFACE_ID_SIZE + 1];
    // ensure NULL-termination
    type[__INTERFACE_TYPE_SIZE] = 0;
    id[__INTERFACE_ID_SIZE] = 0;
    strncpy(type, ih->type, __INTERFACE_TYPE_SIZE);
    strncpy(id, ih->id, __INTERFACE_ID_SIZE);
    if ((fnmatch(type_pattern, type, FNM_NOESCAPE) == 0) &&
	(fnmatch(id_pattern, id, FNM_NOESCAPE) == 0))
    {
      std::string uid = std::string(type) + "::" + id;
      infl->append(ih->type, ih->id, ih->hash, ih->serial,
		   ih->flag_writer_active, ih->num_readers,
		   readers(uid), writer(uid),
		   fawkes::Time(data_ts->timestamp_sec, data_ts->timestamp_usec));
    }
  }

  memmgr->unlock();

  return infl;
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
	  char type[__INTERFACE_TYPE_SIZE + 1] = "Unknown";
	  char id[__INTERFACE_ID_SIZE + 1] = "Invalid";
	  // ensure NULL-termination
	  type[__INTERFACE_TYPE_SIZE] = 0;
	  id[__INTERFACE_ID_SIZE] = 0;
	  std::string uid = "Unknown::Invalid";
	  memmgr->lock();
	  BlackBoardMemoryManager::ChunkIterator cit;
	  for ( cit = memmgr->begin(); cit != memmgr->end(); ++cit ) {
		  interface_header_t *ih = (interface_header_t *)*cit;
		  if (ih->serial == mem_serial) {
			  strncpy(type, ih->type, __INTERFACE_TYPE_SIZE);
			  strncpy(id, ih->id, __INTERFACE_ID_SIZE);
			  break;
		  }
	  }
	  memmgr->unlock();
	  throw BlackBoardNoWritingInstanceException(type, id);
  }
}


void
BlackBoardInterfaceManager::notify_of_data_change(const Interface *interface)
{
  notifier->notify_of_data_change(interface);
}


bool
BlackBoardInterfaceManager::exists_writer(const Interface *interface) const
{
  return (writer_interfaces.find(interface->__mem_serial) != writer_interfaces.end());
}


unsigned int
BlackBoardInterfaceManager::num_readers(const Interface *interface) const
{
  const interface_header_t *ih = (interface_header_t *)interface->__mem_real_ptr;
  return ih->num_readers;
}

std::list<std::string>
BlackBoardInterfaceManager::readers(const Interface *interface) const
{
  std::list<std::string> rv;
  owner_info_.lock();
  LockMap<std::string, OwnerInfo>::const_iterator info;
  if ((info = owner_info_.find(interface->uid())) != owner_info_.end()) {
    std::list<Interface *>::const_iterator i;
    for (i = info->second.readers.begin(); i != info->second.readers.end(); ++i) {
      rv.push_back((*i)->owner());
    }
  }
  owner_info_.unlock();
  return rv;
}


std::string
BlackBoardInterfaceManager::writer(const Interface *interface) const
{
  std::string rv;
  owner_info_.lock();
  LockMap<std::string, OwnerInfo>::const_iterator info;
  if ((info = owner_info_.find(interface->uid())) != owner_info_.end()) {
    if (info->second.writer) {
      rv = info->second.writer->owner();
    }
  }
  owner_info_.unlock();
  return rv;
}


/** Get owners of interfaces who opened for reading.
 * @param uid UID of interface to query for
 * @return list of readers for this interface
 */
std::list<std::string>
BlackBoardInterfaceManager::readers(const std::string &uid) const
{
  std::list<std::string> rv;
  owner_info_.lock();
  LockMap<std::string, OwnerInfo>::const_iterator info;
  if ((info = owner_info_.find(uid)) != owner_info_.end()) {
    std::list<Interface *>::const_iterator i;
    for (i = info->second.readers.begin(); i != info->second.readers.end(); ++i) {
      rv.push_back((*i)->owner());
    }
  }
  owner_info_.unlock();
  return rv;
}


/** Get writer of interface.
 * @param uid UID of interface to query for
 * @return owner name of writing interface instance, or empty string of no writer exists
 */
std::string
BlackBoardInterfaceManager::writer(const std::string &uid) const
{
  std::string rv;
  owner_info_.lock();
  LockMap<std::string, OwnerInfo>::const_iterator info;
  if ((info = owner_info_.find(uid)) != owner_info_.end()) {
    if (info->second.writer) {
      rv = info->second.writer->owner();
    }
  }
  owner_info_.unlock();
  return rv;
}


} // end namespace fawkes
