
/***************************************************************************
 *  interface.cpp - BlackBoard Interface
 *
 *  Created: Mon Oct 09 18:54:50 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
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

#include <interface/interface.h>

#include <interface/mediators/interface_mediator.h>
#include <interface/mediators/message_mediator.h>
#include <core/threading/refc_rwlock.h>
#include <core/exceptions/system.h>
#include <utils/time/clock.h>
#include <utils/time/time.h>

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <typeinfo>
#include <regex.h>

namespace fawkes {

/** @class InterfaceWriteDeniedException <interface/interface.h>
 * This exception is thrown if a write has been attempted on a read-only interface.
 * @see Interface::write()
 * @ingroup Exceptions
 */

/** Constructor.
 * @param type type of the interface which caused the exception
 * @param id id of the interface which caused the exception
 * @param msg additional informative message
 */
InterfaceWriteDeniedException::InterfaceWriteDeniedException(const char *type,
							     const char *id,
							     const char *msg)
  : Exception("This interface instance '%s' of type '%s' is not opened for writing. %s",
	      id, type, msg)
{
}

/** @class InterfaceMessageEnqueueException <interface/interface.h>
 * This exception is thrown if a write has been attempted on a read-only interface.
 * @see Interface::write()
 * @ingroup Exceptions
 */

/** Constructor.
 * @param type type of the interface which caused the exception
 * @param id id of the interface which caused the exception
 */
InterfaceMessageEnqueueException::InterfaceMessageEnqueueException(const char *type,
								   const char *id)
  : Exception("This interface instance '%s' of type '%s' IS opened for writing, but "
	      "messages can only be enqueued on reading interfaces.", id, type)
{
}

/** @class InterfaceInvalidMessageException <interface/interface.h>
 * This exception is thrown if a message has been queued in the interface which is
 * not recognized by the interface.
 * @ingroup Exceptions
 */

/** Constructor.
 * @param interface interface that the invalid message was enqueued to
 * @param message enqueued message
 */
InterfaceInvalidMessageException::InterfaceInvalidMessageException(const Interface *interface,
								   const Message *message)
  : Exception("Message of type '%s' cannot be enqueued in interface of type '%s'",
	      message->type(), interface->type())
{
}


/** @class InterfaceInvalidException <interface/interface.h>
 * This exception is thrown if an interface is invalid and it is attempted to call
 * read()/write().
 * @ingroup Exceptions
 */

/** Constructor.
 * @param interface invalid interface that the operation was tried on
 * @param method the method that was tried to execute
 */
InterfaceInvalidException::InterfaceInvalidException(const Interface *interface,
						     const char *method)
  : Exception("The interface %s (instance serial %u) is invalid. You cannot call %s anymore.",
	      interface->uid(), interface->serial(), method)
{
}


/** @class Interface <interface/interface.h>
 * Base class for all Fawkes BlackBoard interfaces.
 * Never use directly. Use interface generator to create interfaces.
 *
 * Interfaces are identified by a type and an ID. The type is just a textual
 * representation of the class name. The ID identifies a specific instance of this
 * interface type. Additionally each interface has a hash. The hash is an MD5
 * digest of the XML config file that was fed to the interface generator to
 * create the interface. It is used to detect incompatible versions of the same
 * interface type.
 *
 * An interface has an internal timestamp. This timestamp indicates when the
 * data in the interface has been modified last. The timestamp is usually
 * automatically updated. But it some occasions the writer may choose to provide
 * its own timestamp data. This can be useful for example for an interface
 * providing hardware data to give the exact capture time.
 * In the automatic case nothing has to be done manually. The timestamp is
 * updated automatically by calling the write() method if and only if the
 * data in the interface has actually been modified. The reader can call
 * changed() to see if the data changed.
 * In the non-automatic case the writer must first disable automatic timestamping
 * using set_auto_timestamping(). Then it must provide a timestamp everytime
 * before calling write(). Note that setting the timestamp already marks the
 * interface as having changed. So set the timestamp only if the data has
 * changed and the readers should see this.
 *
 * @author Tim Niemueller
 */

/** @var Interface::data_ptr
 * Pointer to local memory storage
 */

/** @var Interface::data_ts
 * Pointer to data casted to timestamp struct. This assumes that the very
 * first two entries are 64 bit wide signed integers containing seconds and
 * microseconds since the Unix epoch.
 */

/** @var Interface::data_size
 * Minimal data size to hold data storage.
 */

/** @var Interface::data_changed
 * Indicator if data has changed.
 * This must be set by all methods that manipulate internal data or the
 * timestamp. Only if set to true a call to write() will update data_ts.
 */

/** @fn bool Interface::message_valid(const Message *message) const = 0
 * Check if the message is valid and can be enqueued.
 * @param message The message to check
 * @return true, if the message is valid and may be enqueued, false otherwise
 *
 * @fn bool Interface::create_message(const char *type) const = 0
 * Create message based on type name.
 * This will create a new message of the given type. The type must be given without
 * the InterfaceName:: prefix but just the plain class name of the message.
 * @param type message type
 * @return message of the given type, empty
 * @exception UnknownTypeException thrown if this interface cannot create a message
 * of the given type.
 *
 * @fn void Interface::copy_values(const Interface *interface) = 0
 * Copy values from another interface.
 * The operation will only succeed if the supplied interface is of the same
 * type as this instance.
 * @param interface interface to copy from
 *
 * @fn const char * Interface::enum_tostring(const char *enumtype, int val) const = 0
 * Convert arbitrary enum value to string.
 * Given the string representation of the enum type and the value this method
 * returns the string representation of the specific value, or the string
 * UNKNOWN if the value is not defined. An exception is thrown if the enum
 * type is invalid.
 * @param enumtype enum type as string
 * @param val value to convert
 * @return string representation of value
 * @exception UnknownTypeException thrown if enumtype is not specified for
 * interface.
 */

/** Constructor */
Interface::Interface()
{
  __write_access = false;
  __rwlock = NULL;
  __valid = true;
  __next_message_id = 0;
  __num_fields = 0;
  __fieldinfo_list   = NULL;
  __messageinfo_list = NULL;
  __clock = Clock::instance();
  __timestamp = new Time(0, 0);
  __local_read_timestamp = new Time(0, 0);
  __auto_timestamping = true;
  data_changed = false;
  memset(__hash, 0, __INTERFACE_HASH_SIZE);
  memset(__hash_printable, 0, __INTERFACE_HASH_SIZE * 2 + 1);

  data_ptr  = NULL;
  data_size = 0;

  __message_queue = new MessageQueue();
}


/** Destructor */
Interface::~Interface()
{
  if ( __rwlock) __rwlock->unref();
  delete __message_queue;
  // free fieldinfo list
  interface_fieldinfo_t *finfol = __fieldinfo_list;
  while ( finfol ) {
    __fieldinfo_list = __fieldinfo_list->next;
    free(finfol);
    finfol = __fieldinfo_list;
  }
  // free messageinfo list
  interface_messageinfo_t *minfol = __messageinfo_list;
  while ( minfol ) {
    __messageinfo_list = __messageinfo_list->next;
    free(minfol);
    minfol = __messageinfo_list;
  }
  delete __timestamp;
}

/** Get interface hash.
 * The interface is a unique version identifier of an interface. It is the has of
 * the input XML file during the generation of the interface. It is meant to be used
 * to ensure that all sides are using the exact same version of an interface.
 * @return constant byte string containing the hash value of hash_size() length
 */
const unsigned char *
Interface::hash() const
{
  return __hash;
}


/** Get printable interface hash.
 * @return printable version of hash()
 */
const char *
Interface::hash_printable() const
{
  return __hash_printable;
}


/** Set hash. Never use directly.
 * @param ihash interface hash
 */
void
Interface::set_hash(unsigned char *ihash)
{
  memcpy(__hash, ihash, __INTERFACE_HASH_SIZE);
  for (size_t s = 0; s < __INTERFACE_HASH_SIZE; ++s) {
    snprintf(&__hash_printable[s*2], 3, "%02X", __hash[s]);
  }
}


/** Add an entry to the field info list.
 * Never use directly, use the interface generator instead. The info list
 * is used for introspection purposes to allow for iterating over all fields
 * of an interface.
 * @param type field type
 * @param name name of the field, this is referenced, not copied
 * @param length length of the field
 * @param value pointer to the value in the data struct
 * @param enumtype name of the enum type, valid only if type == IFT_ENUM.
 */
void
Interface::add_fieldinfo(interface_fieldtype_t type, const char *name,
			 size_t length, void *value, const char *enumtype)
{
  interface_fieldinfo_t *infol = __fieldinfo_list;
  interface_fieldinfo_t *newinfo = (interface_fieldinfo_t *)malloc(sizeof(interface_fieldinfo_t));

  newinfo->type     = type;
  newinfo->enumtype = enumtype;
  newinfo->name     = name;
  newinfo->length   = length;
  newinfo->value    = value;
  newinfo->next     = NULL;

  if ( infol == NULL ) {
    // first entry
    __fieldinfo_list = newinfo;
  } else {
    // append to list
    while ( infol->next != NULL ) {
      infol = infol->next;
    }
    infol->next = newinfo;
  }

  ++__num_fields;
}


/** Add an entry to the message info list.
 * Never use directly, use the interface generator instead. The info list
 * is used for introspection purposes to allow for iterating over all message
 * types of an interface.
 * @param type the type of the message
 */
void
Interface::add_messageinfo(const char *type)
{
  interface_messageinfo_t *infol = __messageinfo_list;
  interface_messageinfo_t *newinfo = (interface_messageinfo_t *)malloc(sizeof(interface_messageinfo_t));

  newinfo->type = type;
  newinfo->next = NULL;

  if ( infol == NULL ) {
    // first entry
    __messageinfo_list = newinfo;
  } else {
    // append to list
    while ( infol->next != NULL ) {
      infol = infol->next;
    }
    infol->next = newinfo;
  }
}


/** Obtain a list of textual representations of the message types
 * available for this interface.
 * @return the message types
 */
std::list<const char *>
Interface::get_message_types()
{
  std::list<const char *> types;
  interface_messageinfo_t *cur = __messageinfo_list;

  while ( cur != NULL ) {
    types.push_back(cur->type);
    cur = cur->next;
  }

  return types;
}


/** Get size of interface hash.
 * Returns the size in bytes of the interface hash. This depends on the used hash.
 * @return size of interface hash string
 */
size_t
Interface::hash_size() const
{
  return __INTERFACE_HASH_SIZE;
}


/** Get data chunk.
 * Use sparsely
 * @return const pointer to the data chunk
 */
const void *
Interface::datachunk() const
{
  return data_ptr;
}


/** Check if this is a writing instance.
 * @return true if this is a writing instance, false otherwise
 */
bool
Interface::is_writer() const
{
  return __write_access;
}


/** Mark this interface invalid.
 * An interface can become invalid, for example if the connection of a RemoteBlackBoard
 * dies. In this case the interface becomes invalid and successive read()/write() calls
 * will throw an InterfaceInvalidException.
 * @param valid true to mark the interface valid or false to mark it invalid
 */
void
Interface::set_validity(bool valid)
{
  __rwlock->lock_for_write();
  __valid = valid;
  __rwlock->unlock();
}


/** Check validity of interface.
 * @return true if interface is valid, false otherwise
 */
bool
Interface::is_valid() const
{
  return __valid;
}


/** Read from BlackBoard into local copy.
 * @exception InterfaceInvalidException thrown if the interface has been marked invalid
 */
void
Interface::read()
{
  __rwlock->lock_for_read();
  if ( __valid ) {
    memcpy(data_ptr, __mem_data_ptr, data_size);
    *__local_read_timestamp = *__timestamp;
    __timestamp->set_time(data_ts->timestamp_sec, data_ts->timestamp_usec);
  } else {
    __rwlock->unlock();
    throw InterfaceInvalidException(this, "read()");
  }
  __rwlock->unlock();
}


/** Write from local copy into BlackBoard memory.
 * @exception InterfaceInvalidException thrown if the interface has been marked invalid
 */
void
Interface::write()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot write.");
  }

  __rwlock->lock_for_write();
  if ( __valid ) {
    memcpy(__mem_data_ptr, data_ptr, data_size);
    if (data_changed) {
      if (__auto_timestamping)  __timestamp->stamp();
      long sec = 0, usec = 0;
      __timestamp->get_timestamp(sec, usec);
      data_ts->timestamp_sec  = sec;
      data_ts->timestamp_usec = usec;
      data_changed = false;
    }
  } else {
    __rwlock->unlock();
    throw InterfaceInvalidException(this, "write()");
  }
  __rwlock->unlock();

  __interface_mediator->notify_of_data_change(this);
}


/** Get data size.
 * @return size in bytes of data segment
 */
unsigned int
Interface::datasize() const
{
  return data_size;
}


/** Set type, ID and UID.
 * Sets type and ID, UID is generated automatically as Type::ID.
 * @param type string, a maxmimum of __INTERFACE_TYPE_SIZE bytes are copied
 * @param ID string, a maxmimum of __INTERFACE_ID_SIZE bytes are copied
 */
void
Interface::set_type_id(const char *type, const char *id)
{
  strncpy(__type, type, __INTERFACE_TYPE_SIZE);
  strncpy(__id, id, __INTERFACE_ID_SIZE);
  snprintf(__uid, __INTERFACE_UID_SIZE, "%s::%s", type, id);
}


/** Set instance serial.
 * @param instance_serial instance serial
 */
void
Interface::set_instance_serial(unsigned short instance_serial)
{
  __instance_serial = instance_serial;
}


/** Set mediators.
 * @param iface_mediator interface mediator
 * @param msg_mediator message mediator.
 */
void
Interface::set_mediators(InterfaceMediator *iface_mediator, MessageMediator *msg_mediator)
{
  __interface_mediator = iface_mediator;
  __message_mediator   = msg_mediator;
}


/** Set memory data.
 * @param serial mem serial
 * @param real_ptr pointer to whole chunk
 * @param data_ptr pointer to data chunk
 */
void
Interface::set_memory(unsigned int serial, void *real_ptr, void *data_ptr)
{
  __mem_serial   = serial;
  __mem_real_ptr = real_ptr;
  __mem_data_ptr = data_ptr;
}


/** Set read/write info.
 * @param write_access true to enable write access, false for read-only
 * @param rwlock read/write lock for this interface
 */
void
Interface::set_readwrite(bool write_access, RefCountRWLock *rwlock)
{
  __write_access = write_access;
  __rwlock       = rwlock;
}


/** Check equality of two interfaces.
 * Two interfaces are the same if their types and identifiers are equal.
 * This does not mean that both interfaces are the very same instance for accessing
 * the BlackBoard. Instead this just means that both instances will access the same
 * chunk of memory in the BlackBoard and the instances MAY be the same.
 * If you want to know if two instances are exactly the same compare the instance
 * serials using the serial() method.
 * @param comp interface to compare current instance with
 * @return true, if interfaces point to the same data, false otherwise
 */
bool
Interface::operator==(Interface &comp) const
{
  return ( (strncmp(__type, comp.__type, sizeof(__type)) == 0) &&
	   (strncmp(__id, comp.__id, sizeof(__id)) == 0) );
}


/** Check if interface is of given type.
 * @param interface_type type to query
 * @return true, if current instance is of given type, false otherwise
 */
bool
Interface::oftype(const char *interface_type) const
{
  return (strncmp(this->__type, interface_type, sizeof(this->__type)) == 0);
}


/** Get type of interface.
 * @return string with the type of the interface.
 */
const char *
Interface::type() const
{
  return __type;
}


/** Get identifier of interface.
 * @return string with the identifier of the interface.
 */
const char *
Interface::id() const
{
  return __id;
}


/** Get unique identifier of interface.
 * As the name suggests this ID denotes a unique memory instance of this interface
 * in the blackboard. It is provided by the system and currently returns a string
 * of the form "type::id", where type is replaced by the type returned by type() and
 * id is the ID returned by id().
 * @return string with the unique identifier of the interface.
 */
const char *
Interface::uid() const
{
  return __uid;
}


/** Get instance serial of interface.
 * @return instance serial of the interface.
 */
unsigned short
Interface::serial() const
{
  return __instance_serial;
}


/** Get memory serial of interface.
 * @return memory serial of interface
 */
unsigned int
Interface::mem_serial() const
{
  return __mem_serial;
}


/** Get timestamp of last write.
 * Note that you need to call read() before this provides useful information.
 * @return timestamp of last write.
 */
const Time *
Interface::timestamp() const
{
  return __timestamp;
}


/** Set timestamp.
 * @param t time stamp to copy time from, if NULL current time is queried
 * from clock.
 */
void
Interface::set_timestamp(const Time *t)
{
  if (!__auto_timestamping) throw Exception("Auto timestamping enabled, cannot "
					    "set explicit timestamp");
  if (!__write_access) throw Exception("Timestamp can only be set on writing "
				       "instance");

  if (t) {
    *__timestamp = t;
  } else {
    __timestamp->stamp();
  }
  data_changed = true;
}


/** Set clock to use for timestamping.
 * @param clock clock to use from now on
 */
void
Interface::set_clock(Clock *clock)
{
  __clock = clock;
  __timestamp->set_clock(clock);
}


/** Enable or disable automated timestamping.
 * @param enabled true to enable automated timestamping, false to disable
 */
void
Interface::set_auto_timestamping(bool enabled)
{
  __auto_timestamping = enabled;
}


/** Check if data has been changed.
 * Note that if the data has been modified this method will return true at least
 * until the next call to read. From then on it will return false if the data has
 * not been modified between the two read() calls and still true otherwise.
 * @return true if data has been changed between the last call to read() and
 * the one before.
 */
bool
Interface::changed() const
{
  return (*__timestamp != __local_read_timestamp);
}


/** Set from a raw data chunk.
 * This allows for setting the interface data from a raw chunk. This is not useful
 * in general but only in rare situations like network transmission. Do not use it unless
 * you really know what you are doing. The method expects the chunk to be exactly of the
 * size returned by datasize(). No check is done, a segfault will most likely occur
 * if you provide invalid data.
 * @param chunk data chunk, must be exactly of the size that is returned by datasize()
 */
void
Interface::set_from_chunk(void *chunk)
{
  // This could be checked but should never happen with our generated interfaces anyway
  // if ( data_ptr == NULL ) throw NullPointerException("Interface not initialized");

  memcpy(data_ptr, chunk, data_size);
}

/** Check if there is a writer for the interface.
 * Use this method to determine if there is any open instance of the interface
 * that is writing to the interface. This can also be the queried interface
 * instance.
 */
bool
Interface::has_writer() const
{
  return __interface_mediator->exists_writer(this);
}


/** Get the number of readers.
 * Use this method to determine how many reading instances of the interface
 * currently exist. If the current instance is a reading instance it will
 * be included in the count number. To determine if you are the last man having
 * this interface you can use the following code:
 * @code
 * // for a writing instance:
 * if ( interface->num_readers == 0 ) {
 *   // we are the last one to have this interface open
 * }
 *
 * // for a reading instance:
 * if ( ! interface->has_writer() && (interface->num_readers() == 0) ) {
 *   // we are the last one to have this interface open
 * }
 * @endcode
 * Note that this can result in a race condition. You have to be registered as
 * a BlackBoardEventListener to be sure that you are really the last.
 */
unsigned int
Interface::num_readers() const
{
  return __interface_mediator->num_readers(this);
}


/** Enqueue message at end of queue.
 * This appends the given message to the queue and transmits the message via the
 * message mediator. The message is afterwards owned by the other side and will be
 * unrefed and freed as soon as it has been processed. If you want to keep this
 * message to read a feedback status you have to reference it _before_ enqueuing
 * it!
 * This can only be called on a reading interface instance.
 * @param message Message to enqueue.
 * @return message id after message has been queued
 * @exception MessageAlreadyQueuedException thrown if the message has already been
 * enqueued to an interface.
 */
unsigned int
Interface::msgq_enqueue(Message *message)
{
  if ( __write_access ) {
    throw InterfaceMessageEnqueueException(__type, __id);
  }
  
  if ( message_valid(message) ) {
    message->set_interface(this);
    message->set_id(next_msg_id());
    // transmit might change the message id!
    __message_mediator->transmit(message);
    unsigned int msgid = message->id();
    message->unref();
    return msgid;
  } else {
    throw InterfaceInvalidMessageException(this, message);
  }
}


/** Enqueue copy of message at end of queue.
 * This method creates a copy of the message and enqueues it. Note that this way
 * you cannot receive status message in the message, because the other side will not
 * use your message instance but a copy instead.
 *
 * This is particularly useful if you call from an environment with automatic garbage
 * collection that does not honor the referencing feature of message but rather just
 * deletes it.
 * This can only be called on a reading interface instance.
 * @param message Message to enqueue.
 * @return message id after message has been queued
 * @exception MessageAlreadyQueuedException thrown if the message has already been
 * enqueued to an interface.
 */
unsigned int
Interface::msgq_enqueue_copy(Message *message)
{
  if ( __write_access ) {
    throw InterfaceMessageEnqueueException(__type, __id);
  }
  if ( message == NULL ) {
    throw NullPointerException("Message may not be NULL");
  }
  
  if ( message_valid(message) ) {
    Message *mcopy = message->clone();
    mcopy->set_interface(this);
    mcopy->set_id(next_msg_id());
    __message_mediator->transmit(mcopy);
    unsigned int msgid = mcopy->id();
    mcopy->unref();
    message->set_id(msgid);
    return msgid;
  } else {
    throw InterfaceInvalidMessageException(this, message);
  }
}


/** Enqueue message.
 * This will enqueue the message without transmitting it via the message mediator.
 * This can only be called on a writing interface instance.
 * @param message message to enqueue
 */
void
Interface::msgq_append(Message *message)
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (append).");
  }

  __message_queue->append(message);
}


/** Remove message from queue.
 * Removes the given message from the queue. Note that if you unref()ed the message
 * after insertion this will most likely delete the object. It is not safe to use the
 * message after removing it from the queue in general. Know what you are doing if
 * you want to use it.
 * This can only be called on a writing interface instance.
 * @param message Message to remove.
 */
void
Interface::msgq_remove(Message *message)
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (remove msg).");
  }

  return __message_queue->remove(message);
}


/** Remove message from queue.
 * Removes message with the given ID from the queue.
 * @param message_id Message ID to remove.
 * This can only be called on a writing interface instance.
 */
void
Interface::msgq_remove(unsigned int message_id)
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (remove id).");
  }

  return __message_queue->remove(message_id);
}


/** Get size of message queue.
 * This can only be called on a writing interface instance.
 * @return number of messages in queue.
 */
unsigned int
Interface::msgq_size()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (size).");
  }

  return __message_queue->size();
}


/** Check if queue is empty.
 * This can only be called on a writing interface instance.
 * @return true if queue is empty, false otherwise
 */
bool
Interface::msgq_empty()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (empty).");
  }

  return __message_queue->empty();
}


/** Flush all messages.
 * Deletes all messages from the queue.
 * This can only be called on a writing interface instance.
 */
void
Interface::msgq_flush()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (flush).");
  }

  __message_queue->flush();
}


/** Lock message queue.
 * Lock the message queue. You have to do this before using the iterator safely.
 * This can only be called on a writing interface instance.
 */
void
Interface::msgq_lock()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (lock).");
  }

  __message_queue->lock();
}


/** Try to lock message queue.
 * Try to lock the message queue. Returns immediately and does not wait for lock.
 * @return true, if the lock has been aquired, false otherwise.
 * @see lock()
 * This can only be called on a writing interface instance.
 */
bool
Interface::msgq_try_lock()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (try_lock).");
  }

  return __message_queue->try_lock();
}


/** Unlock message queue.
 * Give free the lock on the message queue.
 * This can only be called on a writing interface instance.
 */
void
Interface::msgq_unlock()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (unlock).");
  }

  __message_queue->unlock();
}

/** Get start iterator for message queue.
 * Not that you must have locked the queue before this operation!
 * This can only be called on a writing interface instance.
 * @return iterator to begin of message queue.
 * @exception NotLockedException thrown if message queue is not locked during this operation.
 */
MessageQueue::MessageIterator
Interface::msgq_begin()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (begin).");
  }

  return __message_queue->begin();
}


/** Get end iterator for message queue.
 * Not that you must have locked the queue before this operation!
 * This can only be called on a writing interface instance.
 * @return iterator beyond end of message queue.
 * @exception NotLockedException thrown if message queue is not locked during this operation.
 */
MessageQueue::MessageIterator
Interface::msgq_end()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (end).");
  }

  return __message_queue->end();
}


/** Get the first message from the message queue.
 * This can only be called on a writing interface instance.
 * @return first message in queue or NULL if there is none
 */
Message *
Interface::msgq_first()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (first).");
  }

  return __message_queue->first();
}

/** Erase first message from queue.
 * This can only be called on a writing interface instance.
 */
void
Interface::msgq_pop()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id, "Cannot work on message queue on "
					"reading instance of an interface (pop).");
  }

  __message_queue->pop();
}


/** Get iterator over all fields of this interface instance.
 * @return field iterator pointing to the very first value
 */
InterfaceFieldIterator
Interface::fields()
{
  return InterfaceFieldIterator(this, __fieldinfo_list);
}


/** Invalid iterator.
 * @return invalid iterator reprensenting the end.
 */
InterfaceFieldIterator
Interface::fields_end()
{
  return InterfaceFieldIterator();
}


/** Get the number of fields in the interface.
 * @return the number of fields
 */
unsigned int
Interface::num_fields()
{
  return __num_fields;
}


/** Parse UID to type and ID strings.
 * Note that the returned values (type and id) must be freed once they are
 * no longer used. Also verifies lengths of the type and id strings.
 * @param uid UID to parse
 * @param type upon return contains the type part of the UID, must be freed
 * @param id upon return contains the ID part, must be freed
 */
void
Interface::parse_uid(const char *uid, char **type, char **id)
{
  regex_t re;
  int ec = 0;
// Requires in parse_uid()
#define str(s) #s
#define xstr(s) str(s)
  if ((ec = regcomp(&re,
		    "^([a-zA-Z0-9]{1," xstr(__INTERFACE_TYPE_SIZE) "})::"
		    "([a-zA-Z0-9 _\\.-]{1," xstr(__INTERFACE_ID_SIZE) "})$",
		    REG_EXTENDED)) != 0) {
    char errbuf[1024];
    regerror(ec, &re, errbuf, 1024);
    throw Exception("Failed to created regular expression to parse UID (%s)",
		    errbuf);
  }
  regmatch_t matches[3];
  if (regexec(&re, uid, 3, matches, 0) != 0) {
    regfree(&re);
    throw Exception("Failed to match UID %s, format error.", uid);
  }

  *type = strndup(&(uid[matches[1].rm_so]), matches[1].rm_eo - matches[1].rm_so);
  *id   = strndup(&(uid[matches[2].rm_so]), matches[2].rm_eo - matches[2].rm_so);
}

} // end namespace fawkes
