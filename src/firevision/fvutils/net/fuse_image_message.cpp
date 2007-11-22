
/***************************************************************************
 *  fuse_image_message.cpp - FUSE image message encapsulation
 *
 *  Created: Thu Nov 15 15:55:51 2007
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

#include <fvutils/net/fuse_image_message.h>
#include <fvutils/ipc/shm_image.h>

#include <core/exceptions/system.h>

#include <cstdlib>
#include <netinet/in.h>
#include <cstring>

/** @class FuseImageMessage <fvutils/net/fuse_image_message.h>
 * FUSE image message.
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor.
 * @param type message type
 * @param payload payload
 * @param payload_size size of payload
 */
FuseImageMessage::FuseImageMessage(uint32_t type,
				   void *payload, size_t payload_size)
{
  if ( type != FUSE_MT_IMAGE ) {
    throw TypeMismatchException("Type %u != FUSE_MT_IMAGE (%u)", type, FUSE_MT_IMAGE);
  }

  __payload_size = payload_size;
  _msg.header.message_type = htonl(FUSE_MT_IMAGE);
  _msg.header.payload_size = htonl(__payload_size);
  _msg.payload = payload;

  __header = (FUSE_image_message_header_t *)_msg.payload;
  __buffer = (unsigned char *)_msg.payload + sizeof(FUSE_image_message_header_t);

  __buffer_size = ntohl(__header->buffer_size);
}


/** Constructor.
 * Copies data from given buffer.
 * @param b shared memory image buffer to copy image from
 */
FuseImageMessage::FuseImageMessage(SharedMemoryImageBuffer *b)
{
  __buffer_size  = colorspace_buffer_size(b->colorspace(), b->width(), b->height());
  __payload_size = __buffer_size + sizeof(FUSE_image_message_header_t);

  _msg.header.message_type = htonl(FUSE_MT_IMAGE);
  _msg.header.payload_size = htonl(__payload_size);
  _msg.payload = malloc(__payload_size);
  if ( _msg.payload == NULL ) {
    throw OutOfMemoryException("Cannot allocate FuseImageMessage buffer");
  }

  __header = (FUSE_image_message_header_t *)_msg.payload;
  __buffer = (unsigned char *)_msg.payload + sizeof(FUSE_image_message_header_t);

  strncpy(__header->image_id, b->image_id(), IMAGE_ID_MAX_LENGTH);
  __header->format = FUSE_IF_RAW;
  __header->colorspace = htons(b->colorspace());
  __header->reserved = 0;
  __header->width  = htonl(b->width());
  __header->height = htonl(b->height());
  __header->buffer_size = htonl(__buffer_size);

  // b->lock_for_read();
  memcpy(__buffer, b->buffer(), __buffer_size);
  // b->unlock();
}


/** Image buffer.
 * @return image buffer
 */
unsigned char *
FuseImageMessage::buffer() const
{
  return __buffer;
}


/** Get size of buffer.
 * @return size of buffer returned by buffer()
 */
size_t
FuseImageMessage::buffer_size() const
{
  return __buffer_size;
}


/** Get image width.
 * @return width of image in pixels
 */
unsigned int
FuseImageMessage::pixel_width() const
{
  return ntohl(__header->width);
}


/** Get image height.
 * @return height of image in pixels
 */
unsigned int
FuseImageMessage::pixel_height() const
{
  return ntohl(__header->height);
}


/** Get colorspace.
 * @return colorspace
 */
unsigned int
FuseImageMessage::colorspace() const
{
  return ntohs(__header->colorspace);
}


/** Get image format.
 * @return format
 */
unsigned int
FuseImageMessage::format() const
{
  return __header->format;
}
