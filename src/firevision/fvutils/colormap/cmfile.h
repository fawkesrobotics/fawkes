
/**************************************************************************
 *  cmfile.h - FVFF Colormap File Format
 *
 *  Created: Sat Mar 29 12:49:48 2008
 *  Copyright  2005-2008  Tim Niemueller  [www.niemueller.de]
 *
 *  $Id$
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_FVUTILS_COLORMAP_CMFILE_H_
#define __FIREVISION_FVUTILS_COLORMAP_CMFILE_H_

#include <fvutils/fileformat/fvfile.h>
#include <fvutils/colormap/cmfile_block.h>
#include <vector>
#include <string>

class Colormap;

#define CMFILE_MAGIC_TOKEN  0xFF01
#define CMFILE_CUR_VERSION  1

#define CMFILE_TYPE_YUV  1

class ColormapFile : public FireVisionDataFile
{
 public:
  ColormapFile();

  class ColormapBlockVector : public std::vector<ColormapFileBlock *>
  {
   public:
    ~ColormapBlockVector();
  };

  void                   add_colormap(Colormap *colormap);
  ColormapBlockVector *  colormap_blocks();
  Colormap *             get_colormap();

  static std::string     compose_filename(const std::string format);
};

#endif
