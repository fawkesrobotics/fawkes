
/***************************************************************************
 *  message.cpp - Fawkes network message
 *
 *  Created: Tue Nov 21 16:21:28 2006
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

#include <netcomm/fawkes/message.h>

#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>

/** @class FawkesNetworkMessage netcomm/fawkes/message.h
 * Representation of a message that is sent over the network.
 *
 * For the basic format of a message see fawkes_message_t. This class
 * provides access to all of the fields in a convenient manner. Additionally
 * it can handle the client ID, which is either the sender or the recipient
 * of a message (depending if it's in an inbound or outbound queue).
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * Plain constructor. All values initialized to zero, including the
 * client ID.
 */
FawkesNetworkMessage::FawkesNetworkMessage()
{
  memset(&_msg, 0, sizeof(_msg));
  _clid = 0;
}


/** Constructor to set message and client ID.
 * @param clid client ID
 * @param msg reference to message, deep-copied into local message.
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned int clid, fawkes_message_t &msg)
{
  _clid = clid;
  memcpy(&_msg, &msg, sizeof(fawkes_message_t));
}


/** Constructor that only sets message.
 * The client ID is zero.
 * @param msg reference to message, deep-copied into local message.
 */
FawkesNetworkMessage::FawkesNetworkMessage(fawkes_message_t &msg)
{
  _clid = 0;
  memcpy(&_msg, &msg, sizeof(fawkes_message_t));
}


/** Constructor to set single fields.
 * The client ID is set to zero.
 * @param cid component ID
 * @param msg_id message type ID
 * @param payload pointer to payload buffer
 * @param payload_size size of payload buffer
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id,
					   void *payload, unsigned int payload_size)
{
  _msg.header.cid = htons(cid);
  _msg.header.msg_id = htons(msg_id);
  _msg.header.payload_size = htonl(payload_size);
  _msg.payload = payload;
}



/** Constructor to set single fields and allocate memory.
 * The client ID is set to zero. The payload memory is allocated on the heap.
 * @param cid component ID
 * @param msg_id message type ID
 * @param payload_size size of payload buffer
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id,
					   unsigned int payload_size)
{
  _msg.header.cid = htons(cid);
  _msg.header.msg_id = htons(msg_id);
  _msg.header.payload_size = htonl(payload_size);
  _msg.payload = calloc(1, payload_size);
}


/** Constructor to set single fields without payload.
 * The client ID is set to zero.
 * @param cid component ID
 * @param msg_id message type ID
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id)
{
  _msg.header.cid = htons(cid);
  _msg.header.msg_id = htons(msg_id);
  _msg.header.payload_size = 0;
  _msg.payload = NULL;
}


/** Constructor to set single fields and client ID.
 * @param clid client ID
 * @param cid component ID
 * @param msg_id message type ID
 * @param payload pointer to payload buffer
 * @param payload_size size of payload buffer
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned int clid,
					   unsigned short int cid, unsigned short int msg_id,
					   void *payload, unsigned int payload_size)
{
  _clid = clid;
  _msg.header.cid = htons(cid);
  _msg.header.msg_id = htons(msg_id);
  _msg.header.payload_size = htonl(payload_size);
  _msg.payload = payload;
}


/** Constructor to set single fields and client ID without payload.
 * @param clid client ID
 * @param cid component ID
 * @param msg_id message type ID
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned int clid,
					   unsigned short int cid, unsigned short int msg_id)
{
  _clid = clid;
  _msg.header.cid = htons(cid);
  _msg.header.msg_id = htons(msg_id);
  _msg.header.payload_size = 0;
  _msg.payload = NULL;
}


/** Destructor.
 * This destructor also frees the payload buffer if set!
 */
FawkesNetworkMessage::~FawkesNetworkMessage()
{
  if ( _msg.payload != NULL ) {
    free(_msg.payload);
    _msg.payload = NULL;
  }
}


/** Get client ID.
 * @return client ID
 */
unsigned int
FawkesNetworkMessage::clid() const
{
  return _clid;
}


/** Get component ID.
 * @return component ID
 */
unsigned short int
FawkesNetworkMessage::cid() const
{
  return ntohs(_msg.header.cid);
}


/** Get message type ID.
 * @return message type ID
 */
unsigned short int
FawkesNetworkMessage::msgid() const
{
  return ntohs(_msg.header.msg_id);
}


/** Get payload size.
 * @return payload size.
 */
unsigned int
FawkesNetworkMessage::payload_size() const
{
  return ntohl(_msg.header.payload_size);
}


/** Get payload buffer.
 * @return pointer to payload buffer.
 */
void *
FawkesNetworkMessage::payload() const
{
  return _msg.payload;
}


/** Get message reference.
 * @return reference to internal fawkes_message_t, use with care!
 */
const fawkes_message_t &
FawkesNetworkMessage::fmsg() const
{
  return _msg;
}


/** Set client ID.
 * @param clid client ID
 */
void
FawkesNetworkMessage::setClientID(unsigned int clid)
{
  _clid = clid;
}


/** Set component ID.
 * @param cid component ID
 */
void
FawkesNetworkMessage::setComponentID(unsigned short int cid)
{
  _msg.header.cid = htons(cid);
}


/** Set message type ID.
 * @param msg_id message type ID
 */
void
FawkesNetworkMessage::setMessageID(unsigned short int msg_id)
{
  _msg.header.msg_id = htons(msg_id);
}


/** Set payload.
 * @param payload pointer to payload buffer
 * @param payload_size size of payload buffer
 */
void
FawkesNetworkMessage::setPayload(void *payload, unsigned int payload_size)
{
  _msg.payload = payload;
  _msg.header.payload_size = htonl(payload_size);
}


/** Set from message.
 * @param msg reference to message. Content is deep-copied.
 */
void
FawkesNetworkMessage::set(fawkes_message_t &msg)
{
  memcpy(&_msg, &msg, sizeof(fawkes_message_t));
}
