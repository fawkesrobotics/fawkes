
/***************************************************************************
 *  rectfile.h - Rectification info file
 *
 *  Created: Wed Oct 31 11:33:19 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_RECTIFICATION_RECTFILE_H_
#define __FIREVISION_FVUTILS_RECTIFICATION_RECTFILE_H_

#include <fvutils/rectification/rectinfo.h>
#include <list>

class RectificationInfoBlock;

class RectificationInfoFile
{
 public:
  RectificationInfoFile(uint64_t cam_guid);
  ~RectificationInfoFile();

  uint8_t  version();
  uint64_t guid();
  bool     is_big_endian();
  bool     is_little_endian();

  uint8_t  num_blocks();

  void add_rectinfo_block(RectificationInfoBlock *block);
  void write(const char *file_name);
  void read(const char *file_name);
  void clear();

  std::list<RectificationInfoBlock *> &  blocks();

 private:
  rectinfo_header_t  *_header;
  uint64_t            _cam_guid;

  std::list<RectificationInfoBlock *>            info_blocks;
  std::list<RectificationInfoBlock *>::iterator  ibi;
};

#endif
