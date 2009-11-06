
/**************************************************************************
 *  cmfile_block.h - FVFF Colormap File Block
 *
 *  Created: Mon Mar 31 18:01:09 2008
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

#ifndef __FIREVISION_FVUTILS_COLORMAP_CMFILE_BLOCK_H_
#define __FIREVISION_FVUTILS_COLORMAP_CMFILE_BLOCK_H_

#include <fvutils/fileformat/fvfile_block.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Colormap;

class ColormapFileBlock : public FireVisionDataFileBlock
{
 public:
  ColormapFileBlock(unsigned int type, size_t data_size,
		    void *spec_header, size_t spec_header_size);
  ColormapFileBlock(unsigned int type, size_t data_size,
		    size_t spec_header_size);
  ColormapFileBlock(unsigned int type, size_t data_size);
  virtual ~ColormapFileBlock();

 protected:
  ColormapFileBlock(FireVisionDataFileBlock *block);
};

} // end namespace firevision

#endif
