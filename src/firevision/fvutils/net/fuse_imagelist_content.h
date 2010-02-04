
/***************************************************************************
 *  fuse_imagelist_content.h - FUSE image list content encapsulation
 *
 *  Created: Tue Nov 20 14:56:23 2007 (Ella on heat)
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_IMAGELIST_CONTENT_H_
#define __FIREVISION_FVUTILS_NET_FUSE_IMAGELIST_CONTENT_H_

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_message_content.h>
#include <sys/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FuseImageListContent : public FuseMessageContent
{
 public:
  FuseImageListContent();
  FuseImageListContent(uint32_t type, void *payload, size_t payload_size);
  ~FuseImageListContent();

  void add_imageinfo(const char *image_id, colorspace_t colorspace,
		     unsigned int pixel_width, unsigned int pixel_height);

  void                reset_iterator();
  bool                has_next();
  FUSE_imageinfo_t *  next();

  virtual void serialize();

 private:
  fawkes::DynamicBuffer  *__list;
  FUSE_imagelist_message_t __imagelist_msg;
};

} // end namespace firevision

#endif
