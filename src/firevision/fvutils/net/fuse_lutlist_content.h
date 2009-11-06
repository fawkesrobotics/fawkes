
/***************************************************************************
 *  fuse_lutlist_content.h - FUSE LUT list content encapsulation
 *
 *  Created: Wed Nov 21 16:32:31 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_LUTLIST_CONTENT_H_
#define __FIREVISION_FVUTILS_NET_FUSE_LUTLIST_CONTENT_H_

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_message_content.h>
#include <sys/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FuseLutListContent : public FuseMessageContent
{
 public:
  FuseLutListContent();
  FuseLutListContent(uint32_t type, void *payload, size_t payload_size);
  ~FuseLutListContent();

  void add_lutinfo(const char *lut_id,
		   unsigned int width, unsigned int height,
		   unsigned int depth, unsigned int bytes_per_cell);


  void                reset_iterator();
  bool                has_next();
  FUSE_lutinfo_t *    next();

  virtual void        serialize();

 private:
  fawkes::DynamicBuffer  *__list;
  FUSE_lutlist_message_t __lutlist_msg;
};

} // end namespace firevision

#endif
