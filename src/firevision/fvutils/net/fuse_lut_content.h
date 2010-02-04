
/***************************************************************************
 *  fuse_lut_content.h - FUSE LUT content encapsulation
 *
 *  Created: Wed Nov 21 16:47:53 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_LUT_CONTENT_H_
#define __FIREVISION_FVUTILS_NET_FUSE_LUT_CONTENT_H_

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_message_content.h>
#include <sys/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SharedMemoryLookupTable;

class FuseLutContent : public FuseMessageContent
{
 public:
  FuseLutContent(const char *lut_id, void *buffer,
		 unsigned int width, unsigned int height,
		 unsigned int depth, unsigned int bpc);
  FuseLutContent(SharedMemoryLookupTable *l);
  FuseLutContent(uint32_t type, void *payload, size_t payload_size);
  virtual ~FuseLutContent();

  const char *     lut_id() const;
  unsigned char *  buffer() const;
  size_t           buffer_size() const;
  unsigned int     width() const;
  unsigned int     height() const;
  unsigned int     depth() const;
  unsigned int     bytes_per_cell() const;

  virtual void     serialize();

 private:
  char          *__lut_id;
  unsigned char *__buffer;
  size_t         __buffer_size;
  FUSE_lut_message_header_t *__header;
};

} // end namespace firevision

#endif
