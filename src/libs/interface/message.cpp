
/***************************************************************************
 *  message.cpp - BlackBoard message
 *
 *  Generated: Tue Oct 17 00:52:34 2006
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

#include <interface/message.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/exceptions/software.h>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/** @class Message interface/message.h
 * Base class for all messages passed through interfaces in Fawkes BlackBoard.
 * Do not use directly, but instead use the interface generator to generate
 * an interface with accompanying messages.
 *
 * The sender ID of the message is automatically determined from the thread
 * that creates the message.
 *
 * @author Tim Niemueller
 */

/** @var Message::data_ptr
 * Pointer to memory that contains local data. This memory has to be allocated
 * by deriving classes with the approppriate size!
 */

/** @var Message::data_size
 * Size of memory needed to hold all data. This has to be set by deriving classes
 * to the appropriate value.
 */


/** Constructor */
Message::Message()
{
  message_id = 0;
  data_ptr = NULL;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial = 0;
  _status    = Undefined;
  _substatus = 0;
  _sender    = strdup(Thread::current_thread()->name());
  _sender_id = Thread::current_thread_id();
}


/** Copy constructor.
 * @param mesg Message to copy.
 */
Message::Message(Message &mesg)
{
  message_id = 0;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial = 0;
  data_size = mesg.data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, mesg.data_ptr, data_size);
  _status    = Undefined;
  _substatus = 0;
  _sender    = strdup(Thread::current_thread()->name());
  _sender_id = Thread::current_thread_id();
}


/** Copy constructor.
 * @param mesg Message to copy.
 */
Message::Message(Message *mesg)
{
  message_id = 0;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial = 0;
  data_size = mesg->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, mesg->data_ptr, data_size);
  _status    = Undefined;
  _substatus = 0;
  _sender    = strdup(Thread::current_thread()->name());
  _sender_id = Thread::current_thread_id();
}


/** Destructor. */
Message::~Message()
{
  if ( data_ptr != NULL ) {
    free(data_ptr);
    data_ptr = NULL;
  }
  free(_sender);
}


/** Get pointer to data.
 * Avoid usage.
 * @return pointer to internal data
 */
void *
Message::data()
{
  return data_ptr;
}


/** Get size of data.
 * @return size in bytes of data
 */
unsigned int
Message::datasize()
{
  return data_size;
}


/** Assign this message to given message.
 * Data is copied over from message if data sizes are the same.
 * @param m Message to copy
 * @return reference to current instance
 */
Message &
Message::operator=  (const Message & m)
{
  if ( data_size == m.data_size ) {
    memcpy(data_ptr, m.data_ptr, data_size);
  }

  return *this;
}


/** Set the status of the message.
 * Note that this may only be called on the writer side. For efficiency in message
 * handling this is not enforced and the responsibility of the programmer to ensure
 * this.
 * @param status status of the new message.
 */
void
Message::set_status(Message::MessageStatus status)
{
  _status = status;
}


/** Get status of message.
 * @return message status
 */
Message::MessageStatus
Message::status()
{
  return _status;
}


/** Set sub status of a message.
 * The sub status is just an unsigned int which may be defined by the application
 * as needed.
 * @param sub_status new sub status
 */
void
Message::set_sub_status(unsigned int sub_status)
{
  _substatus = sub_status;
}


/** Get sub status.
 * @return sub status
 */
unsigned int
Message::sub_status()
{
  return _substatus;
}


/** Get sender of message.
 * @return name of sending thread
 */
const char *
Message::sender()
{
  return _sender;
}


/** Get ID of sender.
 * @return name of sending thread.
 */
pthread_t
Message::sender_id()
{
  return _sender_id;
}
