
/***************************************************************************
 *  fuse_lutlist_message.h - FUSE LUT list message encapsulation
 *
 *  Created: Wed Nov 21 16:32:31 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_LUTLIST_MESSAGE_H_
#define __FIREVISION_FVUTILS_NET_FUSE_LUTLIST_MESSAGE_H_

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_message.h>
#include <sys/types.h>

class FuseLutListMessage : public FuseNetworkMessage
{
 public:
  FuseLutListMessage();
  FuseLutListMessage(uint32_t type, void *payload, size_t payload_size);
  ~FuseLutListMessage();

  void add_lutinfo(const char *lut_id,
		   unsigned int width, unsigned int height, unsigned int bytes_per_cell);


  void                reset_iterator();
  bool                has_next();
  FUSE_lutinfo_t *  next();

  virtual void pack();

 private:
  DynamicBuffer  *__list;
  FUSE_lutlist_message_t __lutlist_msg;
};

#endif
