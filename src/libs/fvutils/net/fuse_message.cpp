
/***************************************************************************
 *  fuse_message.cpp - FireVision Remote Control Protocol Message Type
 *
 *  Created: Wed Nov 07 13:01:20 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <core/exceptions/software.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_message_content.h>
#include <netinet/in.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace firevision {

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
	content_ = NULL;
}

/** Constructor.
 * @param msg message information to copy
 */
FuseNetworkMessage::FuseNetworkMessage(FUSE_message_t *msg)
{
	memcpy(&_msg, msg, sizeof(FUSE_message_t));
	content_ = NULL;
}

/** Constructor.
 * @param type message type
 * @param payload payload
 * @param payload_size size of payload
 * @param copy_payload if true payload is copied, otherwise payload is referenced
 * and ownership of payload is claimed.
 */
FuseNetworkMessage::FuseNetworkMessage(FUSE_message_type_t type,
                                       void *              payload,
                                       size_t              payload_size,
                                       bool                copy_payload)
{
	content_                 = NULL;
	_msg.header.message_type = htonl(type);
	_msg.header.payload_size = htonl(payload_size);

	if (copy_payload) {
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
	content_                 = NULL;
	_msg.header.message_type = htonl(type);
	_msg.header.payload_size = htonl(0);
	_msg.payload             = NULL;
}

/** Content constructor.
 * Construct a message with complex message content.
 * @param type FUSE message type
 * @param content complex message content.
 */
FuseNetworkMessage::FuseNetworkMessage(FUSE_message_type_t type, FuseMessageContent *content)
{
	content_                 = content;
	_msg.header.message_type = htonl(type);
	_msg.header.payload_size = htonl(0);
	_msg.payload             = NULL;
}

/** Destructor. */
FuseNetworkMessage::~FuseNetworkMessage()
{
	if (content_ == NULL) {
		if (_msg.payload != NULL) {
			free(_msg.payload);
			_msg.payload = NULL;
		}
	} else {
		content_->free_payload();
		delete content_;
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
	if (payload_size > 0xFFFFFFFF) {
		// cannot carry that many bytes
		throw fawkes::OutOfBoundsException("Payload too big", payload_size, 0, 0xFFFFFFFF);
	}
	_msg.payload             = payload;
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

/** Pack data for sending.
 * Use this if any additional packing is needed before sending the data (for
 * example if using a DynamicBuffer).
 */
void
FuseNetworkMessage::pack()
{
	if (content_ != NULL) {
		content_->serialize();
		_msg.payload             = content_->payload();
		_msg.header.payload_size = htonl(content_->payload_size());
	}
}

} // end namespace firevision
