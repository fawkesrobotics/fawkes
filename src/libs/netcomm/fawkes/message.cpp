
/***************************************************************************
 *  message.cpp - Fawkes network message
 *
 *  Created: Tue Nov 21 16:21:28 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>

#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/message_content.h>

#include <netinet/in.h>
#include <cstring>
#include <cstdlib>
#include <cstddef>

namespace fawkes {

/** @class FawkesNetworkMessageTooBigException message.h <netcomm/fawkes/message.h>
 * The given message size exceeds the limit.
 * The message payload can only be of a certain size, which is limited especially
 * by the data type used for the payload size in the header. If you try to assign too
 * much data to a message this exception is thrown.
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param message_size size of the message that is too big
 */
FawkesNetworkMessageTooBigException::FawkesNetworkMessageTooBigException(size_t message_size)
  : Exception("Network message size too big")
{
  fawkes_message_header_t fmh;
  append("Tried to create message of %l bytes, while only %l bytes allowed", message_size,
         sizeof(fmh.payload_size));
}

/** @class FawkesNetworkMessage message.h <netcomm/fawkes/message.h>
 * Representation of a message that is sent over the network.
 *
 * For the basic format of a message see fawkes_message_t. This class
 * provides access to all of the fields in a convenient manner. Additionally
 * it can handle the client ID, which is either the sender or the recipient
 * of a message (depending if it's in an inbound or outbound queue).
 *
 * Note that the message takes over ownership of the payload. This means that it
 * is internally held and freed (using free()) if the message is deleted (if the
 * reference count reaches zero). Because of this you can NOT supply a local variable.
 * The following code is illegal:
 * @code
 * unsigned int u = 0;
 * FawkesNetworkMessage *m = new FawkesNetworkMessage(clid, cid, msgid, &u, sizeof(u));
 * @endcode
 * Rather you have to use the following code:
 * @code
 * unsigned int *u = (unsigned int *)malloc(sizeof(unsigned int));
 * *u = 0;
 * FawkesNetworkMessage *m = new FawkesNetworkMessage(clid, cid, msgid, u, sizeof(unsigned int));
 * @endcode
 *
 * @ingroup NetComm
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
  _content = NULL;
}


/** Constructor to set message and client ID.
 * @param clid client ID
 * @param msg reference to message, deep-copied into local message.
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned int clid, fawkes_message_t &msg)
{
  _content = NULL;
  _clid = clid;
  memcpy(&_msg, &msg, sizeof(fawkes_message_t));
}


/** Constructor that only sets message.
 * The client ID is zero.
 * @param msg reference to message, deep-copied into local message.
 */
FawkesNetworkMessage::FawkesNetworkMessage(fawkes_message_t &msg)
{
  _content = NULL;
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
					   void *payload, size_t payload_size)
{
  _clid = 0;
  _content = NULL;
  if ( payload_size > 0xFFFFFFFF ) {
    // cannot carry that many bytes
    throw FawkesNetworkMessageTooBigException(payload_size);
  }
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
					   size_t payload_size)
{
  _content = NULL;
  _clid = 0;
  if ( payload_size > 0xFFFFFFFF ) {
    // cannot carry that many bytes
    throw FawkesNetworkMessageTooBigException(payload_size);
  }
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
  _content = NULL;
  _clid = 0;
  _msg.header.cid = htons(cid);
  _msg.header.msg_id = htons(msg_id);
  _msg.header.payload_size = 0;
  _msg.payload = NULL;
}


/** Constructor to set single fields.
 * The client ID is set to zero.
 * @param cid component ID
 * @param msg_id message type ID
 * @param content complex content object
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id,
					   FawkesNetworkMessageContent *content)
{
  _content = content;
  _clid = 0;
  _msg.header.cid = htons(cid);
  _msg.header.msg_id = htons(msg_id);
  _msg.header.payload_size = 0;
  _msg.payload = NULL;
}


/** Constructor to set single fields and client ID.
 * @param clid client ID
 * @param cid component ID
 * @param msg_id message type ID
 * @param content complex content object
 */
FawkesNetworkMessage::FawkesNetworkMessage(unsigned int clid,
					   unsigned short int cid, unsigned short int msg_id,
					   FawkesNetworkMessageContent *content)
{
  _content = content;
  _clid = clid;
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
					   void *payload, size_t payload_size)
{
  _content = NULL;
  if ( payload_size > 0xFFFFFFFF ) {
    // cannot carry that many bytes
    throw FawkesNetworkMessageTooBigException(payload_size);
  }
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
  _content = NULL;
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
  if ( _content == NULL ) {
    if ( _msg.payload != NULL ) {
      free(_msg.payload);
      _msg.payload = NULL;
    }
  } else {
    delete _content;
    _content = NULL;
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
size_t
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
FawkesNetworkMessage::set_client_id(unsigned int clid)
{
  _clid = clid;
}


/** Set component ID.
 * @param cid component ID
 */
void
FawkesNetworkMessage::set_component_id(unsigned short int cid)
{
  _msg.header.cid = htons(cid);
}


/** Set message type ID.
 * @param msg_id message type ID
 */
void
FawkesNetworkMessage::set_message_id(unsigned short int msg_id)
{
  _msg.header.msg_id = htons(msg_id);
}


/** Set payload.
 * @param payload pointer to payload buffer
 * @param payload_size size of payload buffer
 */
void
FawkesNetworkMessage::set_payload(void *payload, size_t payload_size)
{
  if ( payload_size > 0xFFFFFFFF ) {
    // cannot carry that many bytes
    throw FawkesNetworkMessageTooBigException(payload_size);
  }
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


/** Set complex message content.
 * @param content complex message content.
 */
void
FawkesNetworkMessage::set_content(FawkesNetworkMessageContent *content)
{
  _content = content;
}


/** Pack data for sending.
 * If complex message sending is required (message content object has been set)
 * then serialize() is called for the content and the message is prepared for
 * sending.
 */
void
FawkesNetworkMessage::pack()
{
  if ( _content != NULL ) {
    _content->serialize();
    _msg.payload = _content->payload();
    _msg.header.payload_size = htonl(_content->payload_size());
  }
}

} // end namespace fawkes
