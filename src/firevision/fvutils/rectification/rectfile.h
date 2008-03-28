
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
#include <fvutils/fileformat/fvfile.h>
#include <vector>

class RectificationInfoBlock;

class RectificationInfoFile : public FireVisionDataFile
{
 public:
  RectificationInfoFile();
  RectificationInfoFile(uint64_t cam_guid, const char *model);
  ~RectificationInfoFile();

  /** Vector that is used for maintaining the rectification info blocks.
   * For instance use RectificationInfoFile::RectInfoBlockVector::iterator as
   * iterator to go through the blocks returned by blocks().
   */
  class RectInfoBlockVector : public std::vector<RectificationInfoBlock *>
  {
    public:
     ~RectInfoBlockVector();
  };

  uint64_t      guid();
  const char *  model();

  void add_rectinfo_block(RectificationInfoBlock *block);

  RectInfoBlockVector  rectinfo_blocks();

  virtual void read(const char *filename);

 private:
  rectinfo_header_t  *_header;
  uint64_t            _cam_guid;
  char               *_model;
};

#endif
