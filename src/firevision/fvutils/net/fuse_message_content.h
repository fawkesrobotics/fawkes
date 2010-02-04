
/***************************************************************************
 *  fuse_message_content.h - FUSE complex message content
 *
 *  Created: Thu Nov 22 17:17:16 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_MESSAGE_CONTENT_H_
#define __FIREVISION_FVUTILS_NET_FUSE_MESSAGE_CONTENT_H_

#include <sys/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FuseMessageContent
{
 public:
  FuseMessageContent();
  virtual ~FuseMessageContent();

  virtual void   serialize() = 0;
  virtual void * payload() const;
  virtual size_t payload_size() const;

  void free_payload();

 protected:
  void copy_payload(size_t offset, void *buf, size_t len);

 protected:
  /** Pointer to payload. */
  void *  _payload;
  /** Payloda size. */
  size_t  _payload_size;

};

} // end namespace firevision

#endif
