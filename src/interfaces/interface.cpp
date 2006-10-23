
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

#include <interfaces/interface.h>

#include <interfaces/mediators/interface_mediator.h>
// include <interfaces/message_queue.h>
#include <core/threading/read_write_lock.h>

#include <string.h>
#include <typeinfo>

/** @class InterfaceWriteDeniedException interfaces/interface.h
 * This exception is thrown if a write has been attempted on a read-only interface.
 * @see Interface::write()
 */

/** @class InterfaceInvalidMessageException interfaces/interface.h
 * This exception is thrown if a message has been queued in the interface which is
 * not recognized by the interface.
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


/** @class Interface interfaces/interface.h
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
  write_access = false;
}


/** Destructor */
Interface::~Interface()
{
  delete rwlock;
}


/** Read from BlackBoard into local copy */
void
Interface::read()
{
  rwlock->lockForRead();
  memcpy(data_ptr, mem_data_ptr, data_size);
  rwlock->unlock();
}


/** Write from local copy into BlackBoard memory. */
void
Interface::write()
{
  if ( ! write_access ) {
    throw InterfaceWriteDeniedException(_type, _id);
  }

  rwlock->lockForWrite();
  memcpy(mem_data_ptr, data_ptr, data_size);
  rwlock->unlock();

  interface_mediator->notifyOfDataChange(this);
}


/*
unsigned int
Interface::enqueue(Message *message)
{
  if ( ! messageValid(message) ) {
    throw InterfaceInvalidMessageException(typeid(message).name(), _type);
  }
  return message_queue->append(message);
}
*/

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
  return ( (strncmp(_type, comp._type, sizeof(_type)) == 0) &&
	   (strncmp(_id, comp._id, sizeof(_id)) == 0) );
}


/** Check if interface is of given type.
 * @param interface_type type to query
 * @return true, if current instance is of given type, false otherwise
 */
bool
Interface::oftype(const char *interface_type) const
{
  return (strncmp(this->_type, interface_type, sizeof(this->_type)) == 0);
}


/** Get type of interface.
 * @return string with the type of the interface.
 */
const char *
Interface::type() const
{
  return _type;
}


/** Get identifier of interface.
 * @return string with the identifier of the interface.
 */
const char *
Interface::id() const
{
  return _id;
}


/** Get serial of interface.
 * @return string with the serial of the interface.
 */
unsigned int
Interface::serial() const
{
  return instance_serial;
}


/** Check if there is a writer for the interface.
 * Use this method to determine if there is any open instance of the interface
 * that is writing to the interface. This can also be the queried interface
 * instance.
 */
bool
Interface::hasWriter() const
{
  return interface_mediator->existsWriter(this);
}


/** @typedef void      (* InterfaceDestroyFunc)  (Interface *interface)
 * @param interface Interface to destroy
 * Interface destructor function for the shared library.
 * Declare and define this function exactly like this:
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
 * Declare and define this function exactly like this:
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
