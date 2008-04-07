
/***************************************************************************
 *  fuse_image_content.h - FUSE image content encapsulation
 *
 *  Created: Thu Nov 15 15:53:32 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_IMAGE_CONTENT_H_
#define __FIREVISION_FVUTILS_NET_FUSE_IMAGE_CONTENT_H_

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_message_content.h>
#include <sys/types.h>

class SharedMemoryImageBuffer;

class FuseImageContent : public FuseMessageContent
{
 public:
  FuseImageContent(SharedMemoryImageBuffer *b);
  FuseImageContent(uint32_t type, void *payload, size_t payload_size);
  FuseImageContent(FUSE_image_format_t image_format, const char *image_id,
		   unsigned char *buffer, size_t buffer_size,
		   colorspace_t colorspace,
		   unsigned int width, unsigned int height);

  unsigned char *  buffer() const;
  size_t           buffer_size() const;
  unsigned int     pixel_width() const;
  unsigned int     pixel_height() const;
  unsigned int     colorspace() const;
  unsigned int     format() const;
  void             decompress(unsigned char *yuv422_planar_buffer, size_t buffer_size);

  virtual void     serialize();

 private:
  unsigned char *__buffer;
  size_t         __buffer_size;
  FUSE_image_message_header_t *__header;
};

#endif
