
/***************************************************************************
 *  rectinfo_block.cpp - Rectification info block encapsulation
 *
 *  Created: Wed Oct 31 14:35:36 2007
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

#include <fvutils/rectification/rectinfo_block.h>
#include <core/exceptions/system.h>
#include <core/exceptions/software.h>

#include <cstdlib>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RectificationInfoBlock <fvutils/rectification/rectinfo_block.h>
 * Rectification info block.
 * This base class defines the basic interface to interact with rectification
 * info blocks. It manages a small memory chunk that may later be used via
 * other recitification information classes in an easy manner. Concrete
 * implementations of a specific block type shall be derived from this
 * class.
 * @author Tim Niemueller
 */

/** @var RectificationInfoBlock::_block_header
 * Rectification block header.
 * This is a pointer to the content-specific block header for rectification info blocks.
 */



/** @fn void RectificationInfoBlock::mapping(uint16_t x, uint16_t y, uint16_t *to_x, uint16_t *to_y) = 0
 * Get mapping (to_x, to_y) for (x, y).
 * This can be used as a general method to access the RectificationInfoBlock mapping.
 * For many models there may be a better (faster) way to access the mapping information.
 * It performance matters (and it most probably will) exploit this and use the
 * provided shortcut.
 * @param x X pixel coordinate to get mapping for
 * @param y Y pixel coordinate to get mapping for
 * @param to_x Upon return contains the X pixel coordinate of the unrectified image
 * @param to_y Upon return contains the Y pixel coordinate of the unrectified image
 */


/** Recommended constructor.
 * With this constructor a chunk of memory is allocated that is sufficient
 * to hold the internal block header and the data of the given size. Note
 * that the size you give is only meant to hold your type specific header
 * and data. Some extra bytes are internally added for the type agnostic
 * block header.
 * @param block_type type of the block as defined per rectinfo_block_type_t
 * @param camera camera identifier
 * @param block_data_size size of the data block, this means only the sum of
 * the size of the type specific header and the data itself, NOT including
 * the type agnostic block header.
 */
RectificationInfoBlock::RectificationInfoBlock(uint8_t block_type,
					       uint8_t camera,
					       size_t block_data_size)
  : FireVisionDataFileBlock(block_type, block_data_size, sizeof(rectinfo_block_header_t))
{
  if ( _data_size > UINT32_MAX ) {
    throw fawkes::OutOfBoundsException("RectInfoBlock: block_data_size is too large",
				       block_data_size, 0, UINT32_MAX);
  }

  _block_header = (rectinfo_block_header_t *)_spec_header;
  _block_header->camera = camera;
}


/** Copy constructor.
 * Copies data from the given FireVisionDataFileBlock. It is assumed that this
 * actually is a rectification info block, check that before calling this
 * method.
 * @param block FireVision data file block
 */
RectificationInfoBlock::RectificationInfoBlock(FireVisionDataFileBlock *block)
  : FireVisionDataFileBlock(block)
{
  _block_header = (rectinfo_block_header_t *)_spec_header;
}


/** Destructor.
 * Destructs the chunk, if and only if _free_block_chunk is true.
 */
RectificationInfoBlock::~RectificationInfoBlock()
{
  _block_header = NULL;
}


/** Get block camera identifier.
 * @return camera identifier
 * @see rectinfo_block_header_t
 */
uint8_t
RectificationInfoBlock::camera() const
{
  if ( _block_header == NULL ) {
    throw fawkes::NullPointerException("No memory chunk loaded for rectinfo block");
  }
  return _block_header->camera;
}

} // end namespace firevision
