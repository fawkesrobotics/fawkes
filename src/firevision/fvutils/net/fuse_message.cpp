
/***************************************************************************
 *  fuse_message.cpp - FireVision Remote Control Protocol Message Type
 *
 *  Created: Wed Nov 07 13:01:20 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <core/exceptions/software.h>
#include <fvutils/net/fuse_message.h>
// include <fvutils/net/fuse_complex_message_content.h>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <netinet/in.h>

/** @class FuseNetworkMessage <fvutils/net/fuse_message.h>
 * FUSE Network Message.
 * This is the basic entity for messages that are sent over the network. Either
 * just use this message to send arbitrary payload or derive this class for more
 * complex behavior or nice encapsulations of messages.
 *
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor. */
FuseNetworkMessage::FuseNetworkMessage()
{
  memset(&_msg, 0, sizeof(_msg));
}


/** Constructor.
 * @param msg message information to copy
 */
FuseNetworkMessage::FuseNetworkMessage(FUSE_message_t *msg)
{
  memcpy(&_msg, msg, sizeof(FUSE_message_t));
}


/** Constructor.
 * @param type message type
 * @param payload payload
 * @param payload_size size of payload
 * @param copy_payload if true payload is copied, otherwise payload is referenced
 * and ownership of payload is claimed.
 */
FuseNetworkMessage::FuseNetworkMessage(FUSE_message_type_t type,
				       void *payload, size_t payload_size,
				       bool copy_payload)
{
  _msg.header.message_type = htonl(type);
  _msg.header.payload_size = htonl(payload_size);

  if ( copy_payload ) {
    _msg.payload = malloc(payload_size);
    memcpy(_msg.payload, payload, payload_size);
  } else {
    _msg.payload = payload;
  }
}


/** Constructor without payload.
 * Constructs message without payload.
 * @param type FUSE message type
 */
FuseNetworkMessage::FuseNetworkMessage(FUSE_message_type_t type)
{
  _msg.header.message_type = htonl(type);
  _msg.header.payload_size = htonl(0);
  _msg.payload = NULL;
}


/** Destructor. */
FuseNetworkMessage::~FuseNetworkMessage()
{
  if ( _msg.payload != NULL ) {
    free(_msg.payload);
    _msg.payload = NULL;
  }
}

/** Get message type.
 * @return message type
 */
uint32_t
FuseNetworkMessage::type() const
{
  return ntohl(_msg.header.message_type);
}


/** Get payload size.
 * @return payload size
 */
size_t
FuseNetworkMessage::payload_size() const
{
  return ntohl(_msg.header.payload_size);
}


/** Get pointer to payload.
 * @return pointer to payload.
 */
void *
FuseNetworkMessage::payload() const
{
  return _msg.payload;
}


/** Get plain message.
 * @return plain message
 */
const FUSE_message_t &
FuseNetworkMessage::fmsg() const
{
  return _msg;
}


/** Set payload.
 * Payload is referenced and ownership claimed.
 * @param payload payload
 * @param payload_size size of payload
 */
void
FuseNetworkMessage::set_payload(void *payload, size_t payload_size)
{
  if ( payload_size > 0xFFFFFFFF ) {
    // cannot carry that many bytes
    throw OutOfBoundsException("Payload too big", payload_size, 0, 0xFFFFFFFF);
  }
  _msg.payload = payload;
  _msg.header.payload_size = htonl(payload_size);
}


/** Set from message.
 * @param msg reference to message. Content is deep-copied.
 */
void
FuseNetworkMessage::set(FUSE_message_t &msg)
{
  memcpy(&_msg, &msg, sizeof(FUSE_message_t));
}


/** Copy payload into payload buffer to a specified offset.
 * This assumes that you have made sure that the buffer is big enough!
 * @param offset offset in _payload where to copy the data to
 * @param buf buffer to copy from
 * @param len number of bytes to copy from buf
 */
void
FuseNetworkMessage::copy_payload(size_t offset, void *buf, size_t len)
{
  void *tmp = (void *)((size_t)(_msg.payload) + offset);
  memcpy(tmp, buf, len);
}

/** Pack data for sending.
 * Use this if any additional packing is needed before sending the data (for
 * example if using a DynamicBuffer).
 */
void
FuseNetworkMessage::pack()
{
}
