
/***************************************************************************
 *  rectinfo_lut_block.h - Rectification info block for 16x16 LUT
 *
 *  Created: Wed Oct 31 14:41:10 2007
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

#ifndef __FIREVISION_FVUTILS_RECTIFICATION_RECTINFO_LUT_BLOCK_H_
#define __FIREVISION_FVUTILS_RECTIFICATION_RECTINFO_LUT_BLOCK_H_

#include <fvutils/rectification/rectinfo_block.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RectificationLutInfoBlock : public RectificationInfoBlock
{
 public:
  RectificationLutInfoBlock(uint16_t width, uint16_t height,
			    uint8_t camera);
  RectificationLutInfoBlock(FireVisionDataFileBlock *block);

  void set_mapping(uint16_t x, uint16_t y,
		   uint16_t to_x, uint16_t to_y);
  virtual void mapping(uint16_t x, uint16_t y,
		       uint16_t *to_x, uint16_t *to_y);

  uint16_t pixel_width();
  uint16_t pixel_height();

  rectinfo_lut_16x16_entry_t *  lut_data();
  
 private:
  rectinfo_lut_16x16_block_header_t *_lut_block_header;
  rectinfo_lut_16x16_entry_t        *_lut_data;

};

} // end namespace firevision

#endif
