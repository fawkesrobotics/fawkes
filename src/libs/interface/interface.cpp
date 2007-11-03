
/***************************************************************************
 *  interface.h - BlackBoard Interface
 *
 *  Generated: Mon Oct 09 18:54:50 2006
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

#include <interface/interface.h>

#include <interface/mediators/interface_mediator.h>
#include <interface/mediators/message_mediator.h>
#include <core/threading/refc_rwlock.h>

#include <string.h>
#include <typeinfo>

/** @class InterfaceWriteDeniedException interface/interface.h
 * This exception is thrown if a write has been attempted on a read-only interface.
 * @see Interface::write()
 * @ingroup Exceptions
 */

/** @class InterfaceInvalidMessageException interface/interface.h
 * This exception is thrown if a message has been queued in the interface which is
 * not recognized by the interface.
 * @ingroup Exceptions
 */

/** Constructor.
 * @param type type of the interface which caused the exception
 * @param id id of the interface which caused the exception
 */
InterfaceWriteDeniedException::InterfaceWriteDeniedException(const char *type,
							     const char *id)
  : Exception()
{
  append("This interface instance '%s' of type '%s' is not opened for writing",
	 id, type);
}

/** Constructor.
 * @param msg_type type of the message which caused the exception
 * @param interface_type type of the interface which caused the exception
 */
InterfaceInvalidMessageException::InterfaceInvalidMessageException(const char *msg_type,
								const char *interface_type)
  : Exception()
{
  append("Message of type '%s' cannot be enqueued in interface of type '%s'",
	 msg_type, interface_type);
}


/** @class Interface interface/interface.h
 * Base class for all Fawkes BlackBoard interfaces.
 * Never use directly. Use interface generator to create interfaces.
 *
 * @author Tim Niemueller
 */

/** @var Interface::data_ptr
 * Pointer to local memory storage
 */

/** @var Interface::data_size
 * Minimal data size to hold data storage.
 */

/** @fn bool Interface::messageValid(const Message *message) const = 0
 * Check if the message is valid and can be enqueued.
 * @param message The message to check
 * @return true, if the message is valid and may be enqueued, false otherwise
 */

/** Constructor */
Interface::Interface()
{
  __write_access = false;
}


/** Destructor */
Interface::~Interface()
{
  __rwlock->unref();
  delete __message_queue;
}


/** Read from BlackBoard into local copy */
void
Interface::read()
{
  __rwlock->lock_for_read();
  memcpy(data_ptr, __mem_data_ptr, data_size);
  __rwlock->unlock();
}


/** Write from local copy into BlackBoard memory. */
void
Interface::write()
{
  if ( ! __write_access ) {
    throw InterfaceWriteDeniedException(__type, __id);
  }

  __rwlock->lock_for_write();
  memcpy(__mem_data_ptr, data_ptr, data_size);
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


/** Check equality of two interfaces.
 * Two interfaces are the same if their types and identifiers are equal.
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


/** Get serial of interface.
 * @return string with the serial of the interface.
 */
unsigned int
Interface::serial() const
{
  return __instance_serial;
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


/** Enqueue message at end of queue.
 * This appends the given message to the queue and transmits the message via the
 * message mediator.
 * @param message Message to enqueue.
 * @return message id after message has been queued
 * @exception MessageAlreadyQueuedException thrown if the message has already been
 * enqueued to an interface.
 */
unsigned int
Interface::msgq_enqueue(Message *message)
{
  message->set_interface(this);
  unsigned int rv = __message_queue->append(message);
  __message_mediator->transmit(message);
  return rv;
}


/** Enqueue message.
 * This will enqueue the message without transmitting it via the message mediator.
 * @param message message to enqueue
 */
unsigned int
Interface::msgq_append(Message *message)
{
  message->set_interface(this);
  return __message_queue->append(message);
}


/** Remove message from queue.
 * Removes the given message from the queue. Note that if you unref()ed the message
 * after insertion this will most likely delete the object. It is not safe to use the
 * message after removing it from the queue in general. Know what you are doing if
 * you want to use it.
 * @param message Message to remove.
 */
void
Interface::msgq_remove(Message *message)
{
  return __message_queue->remove(message);
}


/** Remove message from queue.
 * Removes message with the given ID from the queue.
 * @param message_id Message ID to remove.
 */
void
Interface::msgq_remove(unsigned int message_id)
{
  return __message_queue->remove(message_id);
}


/** Get size of message queue.
 * @return number of messages in queue.
 */
unsigned int
Interface::msgq_size()
{
  return __message_queue->size();
}


/** Check if queue is empty.
 * @return true if queue is empty, false otherwise
 */
bool
Interface::msgq_empty()
{
  return __message_queue->empty();
}


/** Flush all messages.
 * Deletes all messages from the queue.
 */
void
Interface::msgq_flush()
{
  __message_queue->flush();
}


/** Lock message queue.
 * Lock the message queue. You have to do this before using the iterator safely.
 */
void
Interface::msgq_lock()
{
  __message_queue->lock();
}


/** Try to lock message queue.
 * Try to lock the message queue. Returns immediately and does not wait for lock.
 * @return true, if the lock has been aquired, false otherwise.
 * @see lock()
 */
bool
Interface::msgq_try_lock()
{
  return __message_queue->try_lock();
}


/** Unlock message queue.
 * Give free the lock on the message queue.
 */
void
Interface::msgq_unlock()
{
  __message_queue->unlock();
}

/** Get start iterator for message queue.
 * Not that you must have locked the queue before this operation!
 * @return iterator to begin of message queue.
 * @exception NotLockedException thrown if message queue is not locked during this operation.
 */
MessageQueue::MessageIterator
Interface::msgq_begin()
{
  return __message_queue->begin();
}


/** Get end iterator for message queue.
 * Not that you must have locked the queue before this operation!
 * @return iterator beyond end of message queue.
 * @exception NotLockedException thrown if message queue is not locked during this operation.
 */
MessageQueue::MessageIterator
Interface::msgq_end()
{
  return __message_queue->end();
}


/** Get the first message from the message queue.
 * @return first message in queue or NULL if there is none
 */
Message *
Interface::msgq_first()
{
  return __message_queue->first();
}

/** Erase first message from queue.
 */
void
Interface::msgq_pop()
{
  __message_queue->pop();
}


/** @typedef void      (* InterfaceDestroyFunc)  (Interface *interface)
 * @param interface Interface to destroy
 * Interface destructor function for the shared library.
 *
 * This function should never be written by hand but rather the EXPORT_INTERFACE
 * macro should be used.
 *
 * It has to be declared and defined as:
 *
 * @code
 * extern "C"
 * void
 * deleteInterfaceType(Interface *interface)
 * {
 *   delete interface;
 * }
 * @endcode
 * Do not change the type of the function. Do change the name of the function. Replace
 * InterfaceType in the method name with the actual type string of your interface.
 *
 * @relates Interface
 */

/** @typedef Interface *  (* InterfaceFactoryFunc)  (void);
 * Interface generator function for the shared library
 *
 * This function should never be written by hand but rather the EXPORT_INTERFACE
 * macro should be used.
 *
 * It has to be declared and defined as:
 *
 * @code
 * extern "C"
 * Plugin *
 * newInterfaceType()
 * {
 *  return new InterfaceType();
 * }
 * @endcode
 * Do not change the type of the function. Do change the name of the method and the created
 * class type. Change InterfaceType to the type string of your interface.
 * with the name of your plugin derivative.
 *
 * @relates Interface
 */
