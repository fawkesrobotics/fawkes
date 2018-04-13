
/***************************************************************************
 *  fuse_image_content.cpp - FUSE image content encapsulation
 *
 *  Created: Thu Nov 15 15:55:51 2007
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

#include <fvutils/net/fuse_image_content.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include <fvutils/compression/jpeg_decompressor.h>

#include <core/exceptions/system.h>
#include <core/exceptions/software.h>

#include <cstdlib>
#include <netinet/in.h>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FuseImageContent <fvutils/net/fuse_image_content.h>
 * FUSE image content.
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor.
 * @param type message type
 * @param payload payload
 * @param payload_size size of payload
 */
FuseImageContent::FuseImageContent(uint32_t type,
				   void *payload, size_t payload_size)
{
  if ( type != FUSE_MT_IMAGE ) {
    throw fawkes::TypeMismatchException("Type %u != FUSE_MT_IMAGE (%u)", type, FUSE_MT_IMAGE);
  }

  _payload_size = payload_size;
  _payload      = payload;

  __header = (FUSE_image_message_header_t *)_payload;
  __buffer = (unsigned char *)_payload + sizeof(FUSE_image_message_header_t);
  __capture_time = new fawkes::Time(ntohl(__header->capture_time_sec),
				    ntohl(__header->capture_time_usec));

  __buffer_size = ntohl(__header->buffer_size);
}


/** Constructor.
 * Copies data from given buffer.
 * @param b shared memory image buffer to copy image from
 */
FuseImageContent::FuseImageContent(SharedMemoryImageBuffer *b)
{
  __buffer_size  = colorspace_buffer_size(b->colorspace(), b->width(), b->height());
  _payload_size  = __buffer_size + sizeof(FUSE_image_message_header_t);
  _payload = malloc(_payload_size);

  if ( _payload == NULL ) {
    throw fawkes::OutOfMemoryException("Cannot allocate FuseImageContent buffer");
  }

  __header = (FUSE_image_message_header_t *)_payload;
  __buffer = (unsigned char *)_payload + sizeof(FUSE_image_message_header_t);

  strncpy(__header->image_id, b->image_id(), IMAGE_ID_MAX_LENGTH-1);
  __header->format = FUSE_IF_RAW;
  __header->colorspace = htons(b->colorspace());
  __header->reserved = 0;
  __header->width  = htonl(b->width());
  __header->height = htonl(b->height());
  __header->buffer_size = htonl(__buffer_size);

  long int cts = 0, ctus = 0;
  b->capture_time(&cts, &ctus);
  __header->capture_time_sec = htonl(cts);
  __header->capture_time_usec = htonl(ctus);

  __capture_time = NULL;

  b->lock_for_read();
  memcpy(__buffer, b->buffer(), __buffer_size);
  b->unlock();
}


/** Constructor.
 * Copies data from given buffer.
 * @param image_format image format
 * @param image_id image ID
 * @param buffer image buffer, encoded according to image_format
 * @param buffer_size size of buffer in bytes
 * @param colorspace color space
 * @param width width of image in pixels
 * @param height height of image in pixels
 * @param capture_time_sec optional seconds part of the capture time
 * @param capture_time_usec optional microseconds part of the capture time
 */
FuseImageContent::FuseImageContent(FUSE_image_format_t image_format, const char *image_id,
				   unsigned char *buffer, size_t buffer_size,
				   colorspace_t colorspace,
				   unsigned int width, unsigned int height,
				   long int capture_time_sec,
				   long int capture_time_usec)
{
  __buffer_size  = buffer_size;
  _payload_size  = __buffer_size + sizeof(FUSE_image_message_header_t);
  _payload = malloc(_payload_size);

  if ( _payload == NULL ) {
    throw fawkes::OutOfMemoryException("Cannot allocate FuseImageContent buffer");
  }

  __header = (FUSE_image_message_header_t *)_payload;
  __buffer = (unsigned char *)_payload + sizeof(FUSE_image_message_header_t);

  strncpy(__header->image_id, image_id, IMAGE_ID_MAX_LENGTH-1);
  __header->format = image_format;
  __header->colorspace = htons(colorspace);
  __header->reserved = 0;
  __header->width  = htonl(width);
  __header->height = htonl(height);
  __header->buffer_size = htonl(__buffer_size);
  __header->capture_time_sec = htonl(capture_time_sec);
  __header->capture_time_usec = htonl(capture_time_usec);

  __capture_time = NULL;

  memcpy(__buffer, buffer, __buffer_size);
}


/** Destructor. */
FuseImageContent::~FuseImageContent()
{
  delete __capture_time;
}

/** Image buffer.
 * @return image buffer
 */
unsigned char *
FuseImageContent::buffer() const
{
  return __buffer;
}


/** Get size of buffer.
 * @return size of buffer returned by buffer()
 */
size_t
FuseImageContent::buffer_size() const
{
  return __buffer_size;
}


/** Get image width.
 * @return width of image in pixels
 */
unsigned int
FuseImageContent::pixel_width() const
{
  return ntohl(__header->width);
}


/** Get image height.
 * @return height of image in pixels
 */
unsigned int
FuseImageContent::pixel_height() const
{
  return ntohl(__header->height);
}


/** Get colorspace.
 * @return colorspace
 */
unsigned int
FuseImageContent::colorspace() const
{
  return ntohs(__header->colorspace);
}


/** Get image format.
 * @return format
 */
unsigned int
FuseImageContent::format() const
{
  return __header->format;
}


/** Get capture time.
 * @return capture time
 */
fawkes::Time *
FuseImageContent::capture_time() const
{
  if ( ! __capture_time ) {
    __capture_time = new fawkes::Time(ntohl(__header->capture_time_sec),
				      ntohl(__header->capture_time_usec));
  }
  return __capture_time;
}

void
FuseImageContent::serialize()
{
  // Nothing to do here
}


/** Decompress image data.
 * This is a utility method which can be used on clients to decompress compressed
 * image payload. Since every time a new decompressor is created and deleted
 * this method can be slower compared to decompressing the data directly in your
 * application so use with care.
 * @param yuv422_planar_buffer an already allocated buffer where the decompressed image
 * will be stored.
 * @param buffer_size size of yuv422_planar_buffer in bytes. Must be big enough to store
 * a YUV422_PLANAR image of the image dimensions of the compressed data.
 */
void
FuseImageContent::decompress(unsigned char *yuv422_planar_buffer, size_t buffer_size)
{
  if ( buffer_size < colorspace_buffer_size(YUV422_PLANAR, ntohs(__header->width),
					    ntohs(__header->height)) ) {
    throw fawkes::IllegalArgumentException("Supplied buffer is too small\n");
  }
  if ( __header->format != FUSE_IF_JPEG ) {
    JpegImageDecompressor *decompressor = new JpegImageDecompressor();
    decompressor->set_compressed_buffer(__buffer, __buffer_size);
    decompressor->set_decompressed_buffer(yuv422_planar_buffer, buffer_size);
    decompressor->decompress();
    delete decompressor;
  } else {
    convert((colorspace_t)ntohs(__header->colorspace), YUV422_PLANAR,
	    __buffer, yuv422_planar_buffer,
	    ntohs(__header->width), ntohs(__header->height));
  }
}

} // end namespace firevision
