
/***************************************************************************
 *  fuse_lut_message.cpp - FUSE LUT message encapsulation
 *
 *  Created: Wed Nov 21 16:49:18 2007
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

#include <fvutils/net/fuse_lut_message.h>
#include <fvutils/ipc/shm_lut.h>

#include <core/exceptions/system.h>

#include <cstdlib>
#include <netinet/in.h>
#include <cstring>

/** @class FuseLutMessage <fvutils/net/fuse_lut_message.h>
 * FUSE lookup table message.
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor.
 * @param type message type, must be FUSE_MT_LUT
 * @param payload payload
 * @param payload_size size of payload
 * @exception TypeMismatchException thrown if type does not equal FUSE_MT_LUT
 */
FuseLutMessage::FuseLutMessage(uint32_t type,
			       void *payload, size_t payload_size)
{
  if ( type != FUSE_MT_LUT ) {
    throw TypeMismatchException("Type %u != FUSE_MT_LUT (%u)", type, FUSE_MT_LUT);
  }

  __payload_size = payload_size;
  _msg.header.message_type = htonl(FUSE_MT_LUT);
  _msg.header.payload_size = htonl(__payload_size);
  _msg.payload = payload;

  __header = (FUSE_lut_message_header_t *)_msg.payload;
  __buffer = (unsigned char *)_msg.payload + sizeof(FUSE_lut_message_header_t);

  __buffer_size = ntohl(__header->width) * ntohl(__header->height) * ntohl(__header->bytes_per_cell);
}


/** Constructor.
 * @param b lookup table to copy data from
 */
FuseLutMessage::FuseLutMessage(SharedMemoryLookupTable *b)
{
  __buffer_size  = b->width() * b->height() * b->bytes_per_cell();
  __payload_size = __buffer_size + sizeof(FUSE_lut_message_header_t);

  _msg.header.message_type = htonl(FUSE_MT_LUT);
  _msg.header.payload_size = htonl(__payload_size);
  _msg.payload = malloc(__payload_size);
  if ( _msg.payload == NULL ) {
    throw OutOfMemoryException("Cannot allocate FuseLutMessage buffer");
  }

  __header = (FUSE_lut_message_header_t *)_msg.payload;
  __buffer = (unsigned char *)_msg.payload + sizeof(FUSE_lut_message_header_t);

  strncpy(__header->lut_id, b->lut_id(), LUT_ID_MAX_LENGTH);
  __header->width  = htonl(b->width());
  __header->height = htonl(b->height());
  __header->bytes_per_cell = htonl(b->bytes_per_cell());

  // b->lock_for_read(); 
  memcpy(__buffer, b->buffer(), __buffer_size);
  // b->unlock();
}


/** Get buffer.
 * @return buffer
 */
unsigned char *
FuseLutMessage::buffer() const
{
  return __buffer;
}


/** Get buffer size.
 * @return size of buffer returned by buffer()
 */
size_t
FuseLutMessage::buffer_size() const
{
  return __buffer_size;
}


/** Width of LUT.
 * @return width of LUT
 */
unsigned int
FuseLutMessage::width() const
{
  return ntohl(__header->width);
}


/** Height of LUT.
 * @return height of LUT
 */
unsigned int
FuseLutMessage::height() const
{
  return ntohl(__header->height);
}

/** Bytes per cell in LUT.
 * @return Bytes per cell in LUT
 */
unsigned int
FuseLutMessage::bytes_per_cell() const
{
  return ntohs(__header->bytes_per_cell);
}
