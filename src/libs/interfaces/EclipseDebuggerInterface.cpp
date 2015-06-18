
/***************************************************************************
 *  EclipseDebuggerInterface.cpp - Fawkes BlackBoard Interface - EclipseDebuggerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012  Gesche Gierse
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

#include <interfaces/EclipseDebuggerInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class EclipseDebuggerInterface <interfaces/EclipseDebuggerInterface.h>
 * EclipseDebuggerInterface Fawkes BlackBoard Interface.
 * Interface to enable connection between tktools and readylog agent.
 * @ingroup FawkesInterfaces
 */



/** Constructor */
EclipseDebuggerInterface::EclipseDebuggerInterface() : Interface()
{
  data_size = sizeof(EclipseDebuggerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (EclipseDebuggerInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT16, "port", 1, &data->port);
  add_fieldinfo(IFT_STRING, "host", 100, data->host);
  add_messageinfo("ConnectionMessage");
  unsigned char tmp_hash[] = {0xc0, 0x8f, 0x5b, 0xb4, 0xcd, 0xf, 0xe0, 0x88, 0xfd, 0x5d, 0xe4, 0xfe, 0x1, 0xb, 0xa2, 0x83};
  set_hash(tmp_hash);
}

/** Destructor */
EclipseDebuggerInterface::~EclipseDebuggerInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get port value.
 * Port where to connect to
 * @return port value
 */
uint16_t
EclipseDebuggerInterface::port() const
{
  return data->port;
}

/** Get maximum length of port value.
 * @return length of port value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
EclipseDebuggerInterface::maxlenof_port() const
{
  return 1;
}

/** Set port value.
 * Port where to connect to
 * @param new_port new port value
 */
void
EclipseDebuggerInterface::set_port(const uint16_t new_port)
{
  data->port = new_port;
  data_changed = true;
}

/** Get host value.
 * Host where to connect to
 * @return host value
 */
char *
EclipseDebuggerInterface::host() const
{
  return data->host;
}

/** Get maximum length of host value.
 * @return length of host value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
EclipseDebuggerInterface::maxlenof_host() const
{
  return 100;
}

/** Set host value.
 * Host where to connect to
 * @param new_host new host value
 */
void
EclipseDebuggerInterface::set_host(const char * new_host)
{
  strncpy(data->host, new_host, sizeof(data->host));
  data_changed = true;
}

/* =========== message create =========== */
Message *
EclipseDebuggerInterface::create_message(const char *type) const
{
  if ( strncmp("ConnectionMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ConnectionMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
EclipseDebuggerInterface::copy_values(const Interface *other)
{
  const EclipseDebuggerInterface *oi = dynamic_cast<const EclipseDebuggerInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(EclipseDebuggerInterface_data_t));
}

const char *
EclipseDebuggerInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class EclipseDebuggerInterface::ConnectionMessage <interfaces/EclipseDebuggerInterface.h>
 * ConnectionMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
EclipseDebuggerInterface::ConnectionMessage::ConnectionMessage() : Message("ConnectionMessage")
{
  data_size = sizeof(ConnectionMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ConnectionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
EclipseDebuggerInterface::ConnectionMessage::~ConnectionMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
EclipseDebuggerInterface::ConnectionMessage::ConnectionMessage(const ConnectionMessage *m) : Message("ConnectionMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ConnectionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
EclipseDebuggerInterface::ConnectionMessage::clone() const
{
  return new EclipseDebuggerInterface::ConnectionMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
EclipseDebuggerInterface::message_valid(const Message *message) const
{
  const ConnectionMessage *m0 = dynamic_cast<const ConnectionMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(EclipseDebuggerInterface)
/// @endcond


} // end namespace fawkes
