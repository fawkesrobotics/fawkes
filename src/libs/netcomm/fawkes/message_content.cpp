
/***************************************************************************
 *  message_content.cpp - Fawkes network message content
 *
 *  Created: Fri Jun 01 13:31:18 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/fawkes/message_content.h>
#include <core/exceptions/software.h>

#include <cstring>

namespace fawkes {

/** @class FawkesNetworkMessageContent <netcomm/fawkes/message_content.h>
 * Fawkes network message content.
 * Interface for complex Fawkes network messages. Use this type if you want
 * either a nicer interface to your network message or if you need a more
 * complex kind of message type, for example by using DynamicBuffer.
 *
 * Implement all accessor methods that you need and add any data you want.
 * In the end you have to implement serialize() to create a single contiguous
 * buffer that contains all the data that has to be sent. Make _payload point
 * to this buffer and _payload_size contain the size of the buffer.
 *
 * @see DynamicBuffer
 * @ingroup NetComm
 * @author Tim Niemueller
 *
 * @fn void FawkesNetworkMessageContent::serialize() = 0
 * Serialize message content.
 * Generate a single contiguous buffer. Make _payload point to this buffer and
 * _payload_size contain the size of the buffer.
 */

/** Constructor. */
FawkesNetworkMessageContent::FawkesNetworkMessageContent()
{
  _payload = NULL;
  _payload_size = 0;
}


/** Virtual empty destructor. */
FawkesNetworkMessageContent::~FawkesNetworkMessageContent()
{
}


/** Return pointer to payload.
 * @return pointer to payload
 * @exception NullPointerException thrown if _payload does not point to a valid
 * buffer or if _payload_size is zero.
 */
void *
FawkesNetworkMessageContent::payload()
{
  if ( (_payload == NULL) || (_payload_size == 0) ) {
    throw NullPointerException("Payload in network message content may not be NULL");
  }
  return _payload;
}



/** Return payload size
 * @return payload size
 * @exception NullPointerException thrown if _payload does not point to a valid
 * buffer or if _payload_size is zero.
 */
size_t
FawkesNetworkMessageContent::payload_size()
{
  if ( (_payload == NULL) || (_payload_size == 0) ) {
    throw NullPointerException("Payload in network message content may not be NULL");
  }
  return _payload_size;
}


/** Copy payload into payload buffer to a specified offset.
 * This assumes that you have made sure that the buffer is big enough!
 * @param offset offset in _payload where to copy the data to
 * @param buf buffer to copy from
 * @param len number of bytes to copy from buf
 */
void
FawkesNetworkMessageContent::copy_payload(size_t offset, const void *buf, size_t len)
{
  void *tmp = (void *)((size_t)_payload + offset);
  memcpy(tmp, buf, len);
}

} // end namespace fawkes
