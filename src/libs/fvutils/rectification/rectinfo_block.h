
/***************************************************************************
 *  rectinfo_block.h - Rectification info block encapsulation
 *
 *  Created: Wed Oct 31 14:26:34 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

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

#ifndef __FIREVISION_FVUTILS_RECTIFICATION_RECTINFO_BLOCK_H_
#define __FIREVISION_FVUTILS_RECTIFICATION_RECTINFO_BLOCK_H_

#include <fvutils/rectification/rectinfo.h>
#include <fvutils/fileformat/fvfile_block.h>
#include <sys/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RectificationInfoBlock : public FireVisionDataFileBlock
{
 public:
  RectificationInfoBlock(uint8_t block_type, uint8_t camera, size_t block_size);
  RectificationInfoBlock(FireVisionDataFileBlock *block);
  virtual ~RectificationInfoBlock();

  uint8_t  camera() const;

  virtual void mapping(uint16_t x, uint16_t y, uint16_t *to_x, uint16_t *to_y) = 0;

 protected:
  rectinfo_block_header_t  *_block_header;
};

} // end namespace firevision

#endif
