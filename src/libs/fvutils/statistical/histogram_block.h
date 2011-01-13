
/***************************************************************************
 *  histogram_block.h - Histogram block
 *
 *  Created: Sat Mar 29 21:01:49 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __FIREVISIONE_FVUTILS_STATISTICAL_HISTOGRAM_BLOCK_H_
#define __FIREVISIONE_FVUTILS_STATISTICAL_HISTOGRAM_BLOCK_H_

#include <fvutils/fileformat/fvfile_block.h>
#include <fvutils/base/roi.h>
#include <stdint.h>


namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Header for a histogram block. */
typedef struct _histogram_block_header_t
{
  uint16_t width;          /**< the width of the histogram */
  uint16_t height;         /**< the height of the histogram */
  uint16_t depth;          /**< the depth of the histogram */
  uint8_t  object_type;    /**< the type of object the histogram is associated with */
} histogram_block_header_t;


/** The blocktype of a histogram block. */
typedef enum _histogram_block_type_t
{
  FIREVISION_HISTOGRAM_TYPE_16 = 0,    /**< histogram uses 16 bits per cell */
  FIREVISION_HISTOGRAM_TYPE_32 = 1     /**< histogram uses 32 bits per cell */
} histogram_block_type_t;


class HistogramBlock : public FireVisionDataFileBlock
{
 public:
  HistogramBlock(histogram_block_type_t type, hint_t object_type,
		 uint16_t width, uint16_t height, uint16_t depth = 0);
  HistogramBlock(FireVisionDataFileBlock* block);

  virtual ~HistogramBlock();

  uint16_t width() const;
  uint16_t height() const;
  uint16_t depth() const;

  hint_t object_type() const;
  void set_object_type(hint_t object_type);

  void set_data(uint32_t* data);

  void set_value(uint16_t x, uint16_t y, uint32_t val);
  void set_value(uint16_t x, uint16_t y, uint16_t z, uint32_t val);

  uint32_t get_value(uint16_t x, uint16_t y);
  uint32_t get_value(uint16_t x, uint16_t y, uint16_t z);

  void reset();

 private:
  histogram_block_header_t* _block_header;
  uint32_t* _histogram_data;
};

} // end namespace firevision

#endif /* __FIREVISIONE_FVUTILS_STATISTICAL_HISTOGRAM_BLOCK_H_ */
