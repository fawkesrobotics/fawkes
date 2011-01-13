
/***************************************************************************
 *  rectinfo_lut_block.cpp - Rectification info block for 16x16 LUT
 *
 *  Created: Wed Oct 31 15:16:50 2007
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

#include <fvutils/rectification/rectinfo_lut_block.h>

#include <core/exceptions/software.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RectificationLutInfoBlock <fvutils/rectification/rectinfo_lut_block.h>
 * Recitification Lookup Table Block.
 * This class defines a rectification lookup table info block that can be used
 * to define a LUT that maps rectified to unrectified pixels.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param width width of the image
 * @param height height of the image
 * @param camera camera identifier, see rectinfo_camera_t
 */
RectificationLutInfoBlock::RectificationLutInfoBlock(uint16_t width,
						     uint16_t height,
						     uint8_t camera)
  : RectificationInfoBlock(FIREVISION_RECTINFO_TYPE_LUT_16x16,
			   camera,
			   sizeof(rectinfo_lut_16x16_block_header_t) +
			   (width * height * sizeof(rectinfo_lut_16x16_entry_t)))
{
  _lut_block_header = (rectinfo_lut_16x16_block_header_t *)_data;
  _lut_data         = (rectinfo_lut_16x16_entry_t *)((char *)_data +
						     sizeof(rectinfo_lut_16x16_block_header_t));

  _lut_block_header->width  = width;
  _lut_block_header->height = height;
}


/** Copy Constructor.
 * It is assumed that the block actually is a rectification LUT info block. Check that
 * before calling this method.
 * @param block block to copy
 */
RectificationLutInfoBlock::RectificationLutInfoBlock(FireVisionDataFileBlock *block)
  : RectificationInfoBlock(block)
{
  _lut_block_header = (rectinfo_lut_16x16_block_header_t *)_data;
  _lut_data         = (rectinfo_lut_16x16_entry_t *)((char *)_data +
						     sizeof(rectinfo_lut_16x16_block_header_t));
}


void
RectificationLutInfoBlock::mapping(uint16_t x, uint16_t y,
				   uint16_t *to_x, uint16_t *to_y)
{
  if ( x > _lut_block_header->width ) {
    throw OutOfBoundsException("RectLUT X (from)", x, 0, _lut_block_header->width);
  }
  if ( y > _lut_block_header->height ) {
    throw OutOfBoundsException("RectLUT Y (from)", y, 0, _lut_block_header->height);
  }

  *to_x = _lut_data[y * _lut_block_header->width + x].x;
  *to_y = _lut_data[y * _lut_block_header->width + x].y;
}


/** Set mapping.
 * @param x X pixel coordinate to get mapping for
 * @param y Y pixel coordinate to get mapping for
 * @param to_x X pixel coordinate of the unrectified image
 * @param to_y Y pixel coordinate of the unrectified image
 */
void
RectificationLutInfoBlock::set_mapping(uint16_t x, uint16_t y,
				       uint16_t to_x, uint16_t to_y)
{
  if ( x > _lut_block_header->width ) {
    throw OutOfBoundsException("RectLUT X (from)", x, 0, _lut_block_header->width);
  }
  if ( y > _lut_block_header->height ) {
    throw OutOfBoundsException("RectLUT Y (from)", y, 0, _lut_block_header->height);
  }
  if ( to_x > _lut_block_header->width ) {
    throw OutOfBoundsException("RectLUT X (to)", to_x, 0, _lut_block_header->width);
  }
  if ( to_y > _lut_block_header->height ) {
    throw OutOfBoundsException("RectLUT Y (to)", to_y, 0, _lut_block_header->height);
  }

  _lut_data[y * _lut_block_header->width + x].x = to_x;
  _lut_data[y * _lut_block_header->width + x].y = to_y;
}


/** Get width of the LUT.
 * @return width of LUT.
 */
uint16_t
RectificationLutInfoBlock::pixel_width()
{
  return _lut_block_header->width;
}


/** Get height the LUT.
 * @return height of LUT.
 */
uint16_t
RectificationLutInfoBlock::pixel_height()
{
  return _lut_block_header->height;
}


/** Get raw LUT data.
 * Use this to access the LUT.
 * @return pointer to raw LUT data
 */
rectinfo_lut_16x16_entry_t *
RectificationLutInfoBlock::lut_data()
{
  return _lut_data;
}

} // end namespace firevision
