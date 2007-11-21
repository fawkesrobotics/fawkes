
/***************************************************************************
 *  fuse_imagelist_message.h - FUSE image list message encapsulation
 *
 *  Created: Tue Nov 20 14:56:23 2007 (Ella on heat)
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_IMAGELIST_MESSAGE_H_
#define __FIREVISION_FVUTILS_NET_FUSE_IMAGELIST_MESSAGE_H_

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_message.h>
#include <sys/types.h>

class FuseImageListMessage : public FuseNetworkMessage
{
 public:
  FuseImageListMessage();
  FuseImageListMessage(uint32_t type, void *payload, size_t payload_size);
  ~FuseImageListMessage();

  void add_imageinfo(const char *image_id, colorspace_t colorspace,
		     unsigned int pixel_width, unsigned int pixel_height);


  void                reset_iterator();
  bool                has_next();
  FUSE_imageinfo_t *  next();

  virtual void pack();

 private:
  DynamicBuffer  *__list;
  FUSE_imagelist_message_t __imagelist_msg;
};

#endif
