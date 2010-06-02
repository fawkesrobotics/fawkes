
/***************************************************************************
 *  message.cpp - BlackBoard message
 *
 *  Created: Tue Oct 17 00:52:34 2006
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

#include <interface/message.h>
#include <interface/interface.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/exceptions/software.h>
#include <utils/time/time.h>

#include <cstring>
#include <cstdlib>
#include <unistd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Message <interface/message.h>
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
  __fieldinfo_list = NULL;

  __message_id = 0;
  __hops       = 0;
  __enqueued   = false;
  __num_fields = 0;
  data_ptr     = NULL;
  data_ts      = NULL;
  _sender_id   = 0;
  _type        = strdup(type);
  __time_enqueued = new Time();

  _transmit_via_iface              = NULL;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial   = 0;

  Thread *t = Thread::current_thread_noexc();
  if ( t ) {
    _sender_thread_name = strdup(t->name());
  } else {
    _sender_thread_name    = strdup("Unknown");
  }
}


/** Copy constructor.
 * @param mesg Message to copy.
 */
Message::Message(const Message &mesg)
{
  __message_id = 0;
  __hops       = mesg.__hops;
  __enqueued   = false;
  __num_fields = mesg.__num_fields;
  data_size    = mesg.data_size;
  data_ptr     = malloc(data_size);
  data_ts      = (message_data_ts_t *)data_ptr;
  _sender_id   = 0;
  _type        = strdup(mesg._type);
  __time_enqueued = new Time(mesg.__time_enqueued);

  _transmit_via_iface              = NULL;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial   = 0;

  memcpy(data_ptr, mesg.data_ptr, data_size);

  interface_fieldinfo_t  *info_src  = mesg.__fieldinfo_list;
  interface_fieldinfo_t **info_dest = &__fieldinfo_list;
  while ( info_src ) {
    interface_fieldinfo_t *new_info = (interface_fieldinfo_t *)malloc(sizeof(interface_fieldinfo_t));
    memcpy(new_info, info_src, sizeof(interface_fieldinfo_t));
    *info_dest = new_info;

    info_dest = &((*info_dest)->next);
    info_src  = info_src->next;
  }

  Thread *t = Thread::current_thread_noexc();
  if ( t ) {
    _sender_thread_name = strdup(t->name());
  } else {
    _sender_thread_name    = strdup("Unknown");
  }
}


/** Copy constructor.
 * @param mesg Message to copy.
 */
Message::Message(const Message *mesg)
{
  __message_id = 0;
  __hops       = mesg->__hops;
  __enqueued   = false;
  __num_fields = mesg->__num_fields;
  data_size    = mesg->data_size;
  data_ptr     = malloc(data_size);
  data_ts      = (message_data_ts_t *)data_ptr;
  _sender_id   = 0;
  _type        = strdup(mesg->_type);
  _transmit_via_iface              = NULL;
  sender_interface_instance_serial = 0;
  recipient_interface_mem_serial   = 0;
  __time_enqueued = new Time(mesg->__time_enqueued);

  memcpy(data_ptr, mesg->data_ptr, data_size);

  interface_fieldinfo_t  *info_src  = mesg->__fieldinfo_list;
  interface_fieldinfo_t **info_dest = &__fieldinfo_list;
  while ( info_src ) {
    interface_fieldinfo_t *new_info = (interface_fieldinfo_t *)malloc(sizeof(interface_fieldinfo_t));
    memcpy(new_info, info_src, sizeof(interface_fieldinfo_t));
    *info_dest = new_info;

    info_dest = &((*info_dest)->next);
    info_src  = info_src->next;
  }

  Thread *t = Thread::current_thread_noexc();
  if ( t ) {
    _sender_thread_name    = strdup(t->name());
  } else {
    _sender_thread_name    = strdup("Unknown");
  }
}


/** Destructor. */
Message::~Message()
{
  free(_sender_thread_name);
  free(_type);
  delete __time_enqueued;

  interface_fieldinfo_t *infol = __fieldinfo_list;
  while ( infol ) {
    __fieldinfo_list = __fieldinfo_list->next;
    free(infol);
    infol = __fieldinfo_list;
  }
}


/** Get message ID.
 * @return message ID.
 */
unsigned int
Message::id() const
{
  return __message_id;
}


/** Get number of hops.
 * @return number of hops
 */
unsigned int
Message::hops() const
{
  return __hops;
}


/** Set message ID.
 * @param message_id message ID
 */
void
Message::set_id(unsigned int message_id)
{
  __message_id = message_id;
}


/** Set number of hops.
 * @param hops number of hops
 */
void
Message::set_hops(unsigned int hops)
{
  __hops=hops;
}


/** Mark message as being enqueued. */
void
Message::mark_enqueued()
{
  __time_enqueued->stamp();
  long sec = 0, usec = 0;
  __time_enqueued->get_timestamp(sec, usec);
  data_ts->timestamp_sec  = sec;
  data_ts->timestamp_usec = usec;

  __enqueued = true;
}


/** Check is message has been enqueued.
 * @return true if the message has already been enqueued, false otherwise
 */
bool
Message::enqueued() const
{
  return __enqueued;
}


/** Get time when message was enqueued.
 * Note that this assumes synchronized clocks between sender and receiver.
 * Problematic in this regard are remote network connections. For one the
 * system times of the two system can diverge, for the other the clock on
 * only one of the systems may be simulated.
 * @return timestamp when message was enqueued.
 */
const Time *
Message::time_enqueued() const
{
  return __time_enqueued;
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
  __time_enqueued->set_time(data_ts->timestamp_sec, data_ts->timestamp_usec);
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
    __time_enqueued->set_time(data_ts->timestamp_sec, data_ts->timestamp_usec);
  }

  return *this;
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


/** Get iterator over all fields of this interface instance.
 * @return field iterator pointing to the very first value
 */
InterfaceFieldIterator
Message::fields()
{
  return InterfaceFieldIterator(_transmit_via_iface, __fieldinfo_list);
}


/** Invalid iterator.
 * @return invalid iterator reprensenting the end.
 */
InterfaceFieldIterator
Message::fields_end()
{
  return InterfaceFieldIterator();
}


/** Get the number of fields in the message.
 * @return the number of fields
 */
unsigned int
Message::num_fields() const
{
  return __num_fields;
}


/** Clone this message.
 * Shall be implemented by every sub-class to return a message of proper type.
 */
Message *
Message::clone() const
{
  return new Message(this);
}

/** Add an entry to the info list.
 * Never use directly, use the interface generator instead. The info list
 * is used for introspection purposes to allow for iterating over all fields
 * of an interface.
 * @param type field type
 * @param name name of the field, this is referenced, not copied
 * @param length length of the field
 * @param value pointer to the value in the data struct
 * @param enumtype in case the type parameter is enum the name of the enum type
 */
void
Message::add_fieldinfo(interface_fieldtype_t type, const char *name,
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

} // end namespace fawkes
