
/**************************************************************************
 *  cmfile_yuvblock.h - FVFF Colormap File YUV Block
 *
 *  Created: Mon Mar 31 18:08:07 2008
 *  Copyright  2005-2008  Tim Niemueller  [www.niemueller.de]
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_FVUTILS_COLORMAP_CMFILE_YUVBLOCK_H_
#define __FIREVISION_FVUTILS_COLORMAP_CMFILE_YUVBLOCK_H_

#include <fvutils/colormap/cmfile_block.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class YuvColormap;

#pragma pack(push,4)
/** Block header for a YUV block in a ColormapFile. */
typedef struct {
  char      range_from;	/**< Y range from */
  char      range_to;	/**< Y range to */
  uint16_t  reserved;	/**< reserved for future use, padding */
} cmfile_yuvblock_header_t;
#pragma pack(pop)

class ColormapFileYuvBlock : public ColormapFileBlock
{
 public:
  ColormapFileYuvBlock(YuvColormap *cm, unsigned int level = 0);
  ColormapFileYuvBlock(FireVisionDataFileBlock *block);

  unsigned int range_from() const;
  unsigned int range_to() const;

 private:

  YuvColormap  *__cm;
  unsigned int  __level;
  cmfile_yuvblock_header_t *__header;
};

} // end namespace firevision

#endif
