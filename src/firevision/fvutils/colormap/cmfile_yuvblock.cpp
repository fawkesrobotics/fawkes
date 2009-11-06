
/**************************************************************************
 *  cmfile_yuvblock.cpp - FVFF Colormap File YUV Block
 *
 *  Created: Mon Mar 31 18:10:01 2008
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

#include <fvutils/colormap/cmfile_yuvblock.h>
#include <fvutils/colormap/cmfile.h>
#include <fvutils/colormap/yuvcm.h>

#include <core/exceptions/software.h>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColormapFileYuvBlock <fvutils/colormap/cmfile_yuvblock.h>
 * YUV block for colormap file.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cm YUV colormap that this block shall represent.
 * @param level Y level
 */
ColormapFileYuvBlock::ColormapFileYuvBlock(YuvColormap *cm, unsigned int level)
  : ColormapFileBlock(CMFILE_TYPE_YUV, cm->plane_size(), sizeof(cmfile_yuvblock_header_t))
{
  if ( level > cm->depth() ) {
    throw fawkes::OutOfBoundsException("YuvColormap level is out of bounds", level, 0, cm->depth());
  }

  __cm    = cm;
  __level = level;

  __header = (cmfile_yuvblock_header_t *)_spec_header;
  __header->range_from = level * cm->deepness() / cm->depth();
  __header->range_to   = ((level + 1) * cm->deepness() / cm->depth()) - 1;

  memcpy(_data, __cm->get_buffer() + level * cm->plane_size(), _data_size);
}


/** Copy Constructor.
 * It is assumed that the block actually is a rectification LUT info block. Check that
 * before calling this method.
 * @param block block to copy
 */
ColormapFileYuvBlock::ColormapFileYuvBlock(FireVisionDataFileBlock *block)
  : ColormapFileBlock(block)
{
  __header = (cmfile_yuvblock_header_t *)_spec_header;
}


/** Range from value.
 * @return range from value
 */
unsigned int
ColormapFileYuvBlock::range_from() const
{
  return __header->range_from;
}


/** Range to value.
 * @return range to value
 */
unsigned int
ColormapFileYuvBlock::range_to() const
{
  return __header->range_to;
}

} // end namespace firevision
