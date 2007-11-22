
/***************************************************************************
 *  fuse_lut_content.cpp - FUSE LUT content encapsulation
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

#include <fvutils/net/fuse_lut_content.h>
#include <fvutils/ipc/shm_lut.h>

#include <core/exceptions/system.h>
#include <core/exceptions/software.h>

#include <cstdlib>
#include <netinet/in.h>
#include <cstring>

/** @class FuseLutContent <fvutils/net/fuse_lut_content.h>
 * FUSE lookup table content.
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor.
 * @param type content type, must be FUSE_MT_LUT
 * @param payload payload
 * @param payload_size size of payload
 * @exception TypeMismatchException thrown if type does not equal FUSE_MT_LUT
 */
FuseLutContent::FuseLutContent(uint32_t type,
			       void *payload, size_t payload_size)
{
  if ( type != FUSE_MT_LUT ) {
    throw TypeMismatchException("Type %u != FUSE_MT_LUT (%u)", type, FUSE_MT_LUT);
  }

  _payload_size = payload_size;
  _payload = payload;

  __header = (FUSE_lut_message_header_t *)_payload;
  __buffer = (unsigned char *)_payload + sizeof(FUSE_lut_message_header_t);

  __buffer_size = ntohl(__header->width) * ntohl(__header->height) * ntohl(__header->bytes_per_cell);
}


/** Constructor.
 * @param b lookup table to copy data from
 */
FuseLutContent::FuseLutContent(SharedMemoryLookupTable *b)
{
  __buffer_size  = b->width() * b->height() * b->bytes_per_cell();
  _payload_size = __buffer_size + sizeof(FUSE_lut_message_header_t);

  _payload = malloc(_payload_size);
  if ( _payload == NULL ) {
    throw OutOfMemoryException("Cannot allocate FuseLutContent buffer");
  }

  __header = (FUSE_lut_message_header_t *)_payload;
  __buffer = (unsigned char *)_payload + sizeof(FUSE_lut_message_header_t);

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
FuseLutContent::buffer() const
{
  return __buffer;
}


/** Get buffer size.
 * @return size of buffer returned by buffer()
 */
size_t
FuseLutContent::buffer_size() const
{
  return __buffer_size;
}


/** Width of LUT.
 * @return width of LUT
 */
unsigned int
FuseLutContent::width() const
{
  return ntohl(__header->width);
}


/** Height of LUT.
 * @return height of LUT
 */
unsigned int
FuseLutContent::height() const
{
  return ntohl(__header->height);
}

/** Bytes per cell in LUT.
 * @return Bytes per cell in LUT
 */
unsigned int
FuseLutContent::bytes_per_cell() const
{
  return ntohs(__header->bytes_per_cell);
}


void
FuseLutContent::serialize()
{
  // Nothing to do here
}
