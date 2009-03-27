
/**************************************************************************
 *  cmfile_headerblock.h - FVFF Colormap File Header Block
 *
 *  Created: Sun Mar 22 20:00:00 2009
 *  Copyright  2009  Christof Rath <c.rath@student.tugraz.at>
 *
 *  $Id$
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

#ifndef __FIREVISION_FVUTILS_COLORMAP_CMFILE_HEADER_H_
#define __FIREVISION_FVUTILS_COLORMAP_CMFILE_HEADER_H_

#include <fvutils/fileformat/fvfile_block.h>

class Colormap;

#pragma pack(push,4)
/** Block header for a Colormap header block in a ColormapFile. */
typedef struct {
  char      depth;      /**< Y resolution */
  char      width;      /**< U resolution */
  char      height;     /**< V resolution */
  uint16_t  reserved;	/**< reserved for future use, padding */
} cmfile_header_data_t;
#pragma pack(pop)

class ColormapFileHeaderBlock : public FireVisionDataFileBlock
{
 public:
  ColormapFileHeaderBlock(Colormap *cm);
  ColormapFileHeaderBlock(FireVisionDataFileBlock *block);

  unsigned int depth() const;
  unsigned int width() const;
  unsigned int height() const;

 private:

  Colormap              *__cm;
  cmfile_header_data_t  *__header;
};

#endif //__FIREVISION_FVUTILS_COLORMAP_CMFILE_HEADER_H_
