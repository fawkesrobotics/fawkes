
/***************************************************************************
 *  message.cpp - BlackBoard message
 *
 *  Generated: Tue Oct 17 00:52:34 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <interface/message.h>
#include <interface/interface.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>
#include <unistd.h>

/** @class Message interface/message.h
 * Base class for all messages passed through interfaces in Fawkes BlackBoard.
 * Do not use directly, but instead use the interface generator to generate
 * an interface with accompanying messages.
 *
 * The sender ID of the message is automatically determined and is the instance
 * serial of the interface where the message was enqueued using
 * Interface::msgq_enqueue().
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


/** Constructor.
 * @param type string representation of the message type
 */
Message::Message(const char *type)
{
  __message_id = 0;
  data_ptr = NULL;
  _transmit_via_iface = NULL;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial = 0;
  _status    = Undefined;
  _substatus = 0;
  Thread *t = Thread::current_thread_noexc();
  if ( t ) {
    _sender_thread_name = strdup(t->name());
  } else {
    _sender_thread_name    = strdup("Unknown");
  }
  _sender_id = 0;
  _type      = strdup(type);
}


/** Copy constructor.
 * @param mesg Message to copy.
 */
Message::Message(const Message &mesg)
{
  __message_id = 0;
  _transmit_via_iface = NULL;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial = 0;
  data_size = mesg.data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, mesg.data_ptr, data_size);
  _status    = Undefined;
  _substatus = 0;
  Thread *t = Thread::current_thread_noexc();
  if ( t ) {
    _sender_thread_name = strdup(t->name());
  } else {
    _sender_thread_name    = strdup("Unknown");
  }
  _sender_id = 0;
  _type      = strdup(mesg._type);
}


/** Copy constructor.
 * @param mesg Message to copy.
 */
Message::Message(const Message *mesg)
{
  __message_id = 0;
  _transmit_via_iface = NULL;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial = 0;
  data_size = mesg->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, mesg->data_ptr, data_size);
  _status    = Undefined;
  _substatus = 0;
  Thread *t = Thread::current_thread_noexc();
  if ( t ) {
    _sender_thread_name    = strdup(t->name());
  } else {
    _sender_thread_name    = strdup("Unknown");
  }
  _sender_id = 0;
  _type      = strdup(mesg->_type);
}


/** Destructor. */
Message::~Message()
{
  free(_sender_thread_name);
  free(_type);
}


/** Get message ID.
 * @return message ID.
 */
unsigned int
Message::id() const
{
  return __message_id;
}


/** Set message ID.
 * @param message_id message ID
 */
void
Message::set_id(unsigned int message_id)
{
  __message_id = message_id;
}


/** Get recipient memory serial.
 * @return Interface memory serial of the recipient interface.
 */
unsigned int
Message::recipient() const
{
  return recipient_interface_mem_serial;
}

/** Get pointer to data.
 * Avoid usage.
 * @return pointer to internal data
 */
const void *
Message::datachunk() const
{
  return data_ptr;
}


/** Get size of data.
 * @return size in bytes of data
 */
unsigned int
Message::datasize() const
{
  return data_size;
}


/** Set from raw data chunk.
 * This sets the internal storage to the given chunk. The chunk must be exactly
 * of the size returned by datasize().
 * @param chunk chunk containing the data exactly of the size returned by datasize()
 */
void
Message::set_from_chunk(const void *chunk)
{
  memcpy(data_ptr, chunk, data_size);
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
Message::status() const
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
Message::sub_status() const
{
  return _substatus;
}


/** Get sender of message.
 * @return name of sending thread
 */
const char *
Message::sender_thread_name() const
{
  return _sender_thread_name;
}


/** Get ID of sender.
 * @return name of sending thread.
 */
unsigned int
Message::sender_id() const
{
  return _sender_id;
}


/** Set transmitting interface.
 * Called by Message Manager
 * @param iface transmitting interface
 */
void
Message::set_interface(Interface *iface)
{
  _transmit_via_iface = iface;
  _sender_id = iface->serial();
  recipient_interface_mem_serial = iface->mem_serial();
}


/** Get transmitting interface.
 * @return transmitting interface, or NULL if message has not been enqueued, yet.
 */
Interface *
Message::interface() const
{
  return _transmit_via_iface;
}


/** Get message type.
 * @return textual representation of the interface type
 */
const char *
Message::type() const
{
  return _type;
}


/** Clone this message.
 * Shall be implemented by every sub-class to return a message of proper type.
 */
Message *
Message::clone() const
{
  return new Message(this);
}
