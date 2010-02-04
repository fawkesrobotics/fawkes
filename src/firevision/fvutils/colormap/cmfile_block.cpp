
/**************************************************************************
 *  cmfile_block.cpp - FVFF Colormap File Block
 *
 *  Created: Mon Mar 31 18:06:17 2008
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

#include <fvutils/colormap/cmfile_block.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColormapFileBlock <fvutils/colormap/cmfile_block.h>
 * FireVision data file block for colormap files.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param type block type, content specific
 * @param data_size size of the data segment
 * @param spec_header content-specific header
 * @param spec_header_size size of spec_header in bytes
 */
ColormapFileBlock::ColormapFileBlock(unsigned int type, size_t data_size,
				     void *spec_header, size_t spec_header_size)
  : FireVisionDataFileBlock(type, data_size, spec_header, spec_header_size)
{
}


/** Constructor.
 * @param type block type, content specific
 * @param data_size size of the data segment
 * @param spec_header_size size of spec_header in bytes
 */
ColormapFileBlock::ColormapFileBlock(unsigned int type, size_t data_size,
				     size_t spec_header_size)
  : FireVisionDataFileBlock(type, data_size, spec_header_size)
{
}


/** Constructor.
 * @param type block type, content specific
 * @param data_size size of the data segment
 */
ColormapFileBlock::ColormapFileBlock(unsigned int type, size_t data_size)
  : FireVisionDataFileBlock(type, data_size)
{
}


/** Shallow copy constructor.
 * This creates a shallow copy of the given block. "Shallow" means that the data is not
 * copied but referenced. This instance is only valid as long as the original instance
 * still exists.
 * @param block block to copy
 */
ColormapFileBlock::ColormapFileBlock(FireVisionDataFileBlock *block)
  : FireVisionDataFileBlock(block)
{
}


/** Virtual empty destructor. */
ColormapFileBlock::~ColormapFileBlock()
{
}

} // end namespace firevision
