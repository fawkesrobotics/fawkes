
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

  header_ = (FUSE_image_message_header_t *)_payload;
  buffer_ = (unsigned char *)_payload + sizeof(FUSE_image_message_header_t);
  capture_time_ = new fawkes::Time(ntohl(header_->capture_time_sec),
				    ntohl(header_->capture_time_usec));

  buffer_size_ = ntohl(header_->buffer_size);
}


/** Constructor.
 * Copies data from given buffer.
 * @param b shared memory image buffer to copy image from
 */
FuseImageContent::FuseImageContent(SharedMemoryImageBuffer *b)
{
  buffer_size_  = colorspace_buffer_size(b->colorspace(), b->width(), b->height());
  _payload_size  = buffer_size_ + sizeof(FUSE_image_message_header_t);
  _payload = malloc(_payload_size);

  if ( _payload == NULL ) {
    throw fawkes::OutOfMemoryException("Cannot allocate FuseImageContent buffer");
  }

  header_ = (FUSE_image_message_header_t *)_payload;
  buffer_ = (unsigned char *)_payload + sizeof(FUSE_image_message_header_t);

  strncpy(header_->image_id, b->image_id(), IMAGE_ID_MAX_LENGTH-1);
  header_->format = FUSE_IF_RAW;
  header_->colorspace = htons(b->colorspace());
  header_->reserved = 0;
  header_->width  = htonl(b->width());
  header_->height = htonl(b->height());
  header_->buffer_size = htonl(buffer_size_);

  long int cts = 0, ctus = 0;
  b->capture_time(&cts, &ctus);
  header_->capture_time_sec = htonl(cts);
  header_->capture_time_usec = htonl(ctus);

  capture_time_ = NULL;

  b->lock_for_read();
  memcpy(buffer_, b->buffer(), buffer_size_);
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
  buffer_size_  = buffer_size;
  _payload_size  = buffer_size_ + sizeof(FUSE_image_message_header_t);
  _payload = malloc(_payload_size);

  if ( _payload == NULL ) {
    throw fawkes::OutOfMemoryException("Cannot allocate FuseImageContent buffer");
  }

  header_ = (FUSE_image_message_header_t *)_payload;
  buffer_ = (unsigned char *)_payload + sizeof(FUSE_image_message_header_t);

  strncpy(header_->image_id, image_id, IMAGE_ID_MAX_LENGTH-1);
  header_->format = image_format;
  header_->colorspace = htons(colorspace);
  header_->reserved = 0;
  header_->width  = htonl(width);
  header_->height = htonl(height);
  header_->buffer_size = htonl(buffer_size_);
  header_->capture_time_sec = htonl(capture_time_sec);
  header_->capture_time_usec = htonl(capture_time_usec);

  capture_time_ = NULL;

  memcpy(buffer_, buffer, buffer_size_);
}


/** Destructor. */
FuseImageContent::~FuseImageContent()
{
  delete capture_time_;
}

/** Image buffer.
 * @return image buffer
 */
unsigned char *
FuseImageContent::buffer() const
{
  return buffer_;
}


/** Get size of buffer.
 * @return size of buffer returned by buffer()
 */
size_t
FuseImageContent::buffer_size() const
{
  return buffer_size_;
}


/** Get image width.
 * @return width of image in pixels
 */
unsigned int
FuseImageContent::pixel_width() const
{
  return ntohl(header_->width);
}


/** Get image height.
 * @return height of image in pixels
 */
unsigned int
FuseImageContent::pixel_height() const
{
  return ntohl(header_->height);
}


/** Get colorspace.
 * @return colorspace
 */
unsigned int
FuseImageContent::colorspace() const
{
  return ntohs(header_->colorspace);
}


/** Get image format.
 * @return format
 */
unsigned int
FuseImageContent::format() const
{
  return header_->format;
}


/** Get capture time.
 * @return capture time
 */
fawkes::Time *
FuseImageContent::capture_time() const
{
  if ( ! capture_time_ ) {
    capture_time_ = new fawkes::Time(ntohl(header_->capture_time_sec),
				      ntohl(header_->capture_time_usec));
  }
  return capture_time_;
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
  if ( buffer_size < colorspace_buffer_size(YUV422_PLANAR, ntohs(header_->width),
					    ntohs(header_->height)) ) {
    throw fawkes::IllegalArgumentException("Supplied buffer is too small\n");
  }
  if ( header_->format != FUSE_IF_JPEG ) {
    JpegImageDecompressor *decompressor = new JpegImageDecompressor();
    decompressor->set_compressed_buffer(buffer_, buffer_size_);
    decompressor->set_decompressed_buffer(yuv422_planar_buffer, buffer_size);
    decompressor->decompress();
    delete decompressor;
  } else {
    convert((colorspace_t)ntohs(header_->colorspace), YUV422_PLANAR,
	    buffer_, yuv422_planar_buffer,
	    ntohs(header_->width), ntohs(header_->height));
  }
}

} // end namespace firevision
