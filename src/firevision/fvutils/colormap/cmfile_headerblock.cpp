
/**************************************************************************
 *  cmfile_headerblock.cpp - FVFF Colormap File Header Block
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

#include <fvutils/colormap/cmfile_headerblock.h>
#include <fvutils/colormap/cmfile.h>
#include <fvutils/colormap/colormap.h>


/** @class ColormapFileHeaderBlock cmfile_headerblock.h <fvutils/colormap/cmfile_headerblock.h>
 * Header block for colormap file.
 *
 * @author Christof Rath
 */

/** Constructor.
 * @param cm colormap that this block shall represent.
 */
ColormapFileHeaderBlock::ColormapFileHeaderBlock(Colormap *cm)
  : FireVisionDataFileBlock(CMFILE_TYPE_HEADER, 0, sizeof(cmfile_header_data_t))
{
  __cm    = cm;

  __header = (cmfile_header_data_t *)_spec_header;
  __header->depth  = cm->depth();
  __header->width  = cm->width();
  __header->height = cm->height();
}


/** Copy Constructor.
 * It is assumed that the block actually is a rectification LUT info block. Check that
 * before calling this method.
 * @param block block to copy
 */
ColormapFileHeaderBlock::ColormapFileHeaderBlock(FireVisionDataFileBlock *block)
  : FireVisionDataFileBlock(block)
{
  __header = (cmfile_header_data_t *)_spec_header;
  __cm     = NULL;
}

/** Y resolution of the block
 * @return Y resolution
 */
unsigned int
ColormapFileHeaderBlock::depth() const
{
  return __header->depth;
}


/** U resolution of the block
 * @return U resolution
 */
unsigned int
ColormapFileHeaderBlock::width() const
{
  return __header->width;
}


/** V resolution of the block
 * @return V resolution
 */
unsigned int
ColormapFileHeaderBlock::height() const
{
  return __header->height;
}
