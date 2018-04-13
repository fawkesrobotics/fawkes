
/***************************************************************************
 *  fuse_lut_content.cpp - FUSE LUT content encapsulation
 *
 *  Created: Wed Nov 21 16:49:18 2007
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

#include <fvutils/net/fuse_lut_content.h>
#include <fvutils/ipc/shm_lut.h>

#include <core/exceptions/system.h>
#include <core/exceptions/software.h>

#include <cstdlib>
#include <netinet/in.h>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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
  if ( (type != FUSE_MT_LUT) && (type != FUSE_MT_SET_LUT) ) {
    throw fawkes::TypeMismatchException("Type %u != FUSE_MT_LUT/FUSE_MT_SET_LUT (%u/%u)",
					type, FUSE_MT_LUT, FUSE_MT_SET_LUT);
  }

  _payload_size = payload_size;
  _payload = payload;
  

  __header = (FUSE_lut_message_header_t *)_payload;
  __buffer = (unsigned char *)_payload + sizeof(FUSE_lut_message_header_t);

  __lut_id = (char *)malloc(LUT_ID_MAX_LENGTH + 1);
  __lut_id[LUT_ID_MAX_LENGTH] = 0;
  strncpy(__lut_id, __header->lut_id, LUT_ID_MAX_LENGTH);

  __buffer_size = ntohl(__header->width) * ntohl(__header->height) *
                  ntohl(__header->depth) * ntohl(__header->bytes_per_cell);
}


/** Constructor.
 * @param b lookup table to copy data from
 */
FuseLutContent::FuseLutContent(SharedMemoryLookupTable *b)
{
  __buffer_size  = b->width() * b->height() * b->depth() * b->bytes_per_cell();
  _payload_size = __buffer_size + sizeof(FUSE_lut_message_header_t);

  _payload = malloc(_payload_size);
  if ( _payload == NULL ) {
    throw fawkes::OutOfMemoryException("Cannot allocate FuseLutContent buffer");
  }

  __header = (FUSE_lut_message_header_t *)_payload;
  __buffer = (unsigned char *)_payload + sizeof(FUSE_lut_message_header_t);

  strncpy(__header->lut_id, b->lut_id(), LUT_ID_MAX_LENGTH-1);
  __header->width  = htonl(b->width());
  __header->height = htonl(b->height());
  __header->depth  = htonl(b->depth());
  __header->bytes_per_cell = htonl(b->bytes_per_cell());
  __lut_id = strdup(b->lut_id());

  // b->lock_for_read(); 
  memcpy(__buffer, b->buffer(), __buffer_size);
  // b->unlock();
}


/** Constructor.
 * Create a brand new FuseLutContent from a raw buffer.
 * @param lut_id LUT ID
 * @param buffer buffer that holds the LUT data
 * @param width LUT width
 * @param height LUT height
 * @param depth LUT depth
 * @param bpc LUT bytes per cell
 */
FuseLutContent::FuseLutContent(const char *lut_id, void *buffer,
			       unsigned int width, unsigned int height,
			       unsigned int depth, unsigned int bpc)
{
  __buffer_size  = width * height * depth * bpc;
  _payload_size = __buffer_size + sizeof(FUSE_lut_message_header_t);

  _payload = malloc(_payload_size);
  if ( _payload == NULL ) {
    throw fawkes::OutOfMemoryException("Cannot allocate FuseLutContent buffer");
  }

  __header = (FUSE_lut_message_header_t *)_payload;
  __buffer = (unsigned char *)_payload + sizeof(FUSE_lut_message_header_t);

  strncpy(__header->lut_id, lut_id, LUT_ID_MAX_LENGTH-1);
  __header->width  = htonl(width);
  __header->height = htonl(height);
  __header->depth  = htonl(depth);
  __header->bytes_per_cell = htonl(bpc);
  __lut_id = strdup(lut_id);

  memcpy(__buffer, buffer, __buffer_size);
}


FuseLutContent::~FuseLutContent()
{
  free(__lut_id);
}


/** Get LUT ID.
 * @return LUT ID
 */
const char *
FuseLutContent::lut_id() const
{
  return __lut_id;
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

/** Depth of LUT.
 * @return depth of LUT
 */
unsigned int
FuseLutContent::depth() const
{
  return ntohl(__header->depth);
}


/** Bytes per cell in LUT.
 * @return Bytes per cell in LUT
 */
unsigned int
FuseLutContent::bytes_per_cell() const
{
  return ntohl(__header->bytes_per_cell);
}


void
FuseLutContent::serialize()
{
  // Nothing to do here
}

} // end namespace firevision
