
/***************************************************************************
 *  fuse_lut_content.h - FUSE LUT content encapsulation
 *
 *  Created: Wed Nov 21 16:47:53 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_LUT_CONTENT_H_
#define __FIREVISION_FVUTILS_NET_FUSE_LUT_CONTENT_H_

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_message_content.h>
#include <sys/types.h>

class SharedMemoryLookupTable;

class FuseLutContent : public FuseMessageContent
{
 public:
  FuseLutContent(SharedMemoryLookupTable *l);
  FuseLutContent(uint32_t type, void *payload, size_t payload_size);

  unsigned char *  buffer() const;
  size_t           buffer_size() const;
  unsigned int     width() const;
  unsigned int     height() const;
  unsigned int     bytes_per_cell() const;

  virtual void     serialize();

 private:
  unsigned char *__buffer;
  size_t         __buffer_size;
  FUSE_lut_message_header_t *__header;
};

#endif
