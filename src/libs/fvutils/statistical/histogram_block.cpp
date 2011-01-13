
/***************************************************************************
 *  histogram_block.cpp - Histogram block
 *
 *  Created: Sat Mar 29 21:01:35 2008
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

#include <fvutils/statistical/histogram_block.h>
#include <core/exceptions/software.h>
#include <cstring>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class HistogramBlock <fvutils/statistical/histogram_block.h>
 * This class defines a file block for histograms. Additionally, the very basic routines
 * to acccess and manipulate data in the histograms are provided.
 * @author Daniel Beck
 */


/** Constructor.
 * @param type the type of the histogram block
 * @param object_type the object type this histogram is meant for (e.g, ball)
 * @param width the width of the histogram
 * @param height the height of the histogram
 * @param depth the depth of the histogram
 */
HistogramBlock::HistogramBlock(histogram_block_type_t type, hint_t object_type,
			       uint16_t width, uint16_t height, uint16_t depth)
  : FireVisionDataFileBlock(type, width * height * depth * sizeof(uint32_t), 
			    sizeof(histogram_block_header_t))
{
  _block_header = (histogram_block_header_t*) _spec_header;
  _block_header->width = width;
  _block_header->height = height;
  _block_header->depth = depth;
  _block_header->object_type = object_type;

  _histogram_data = (uint32_t*) _data;
}

/** Copy constructor.
 * @param block another block
 */
HistogramBlock::HistogramBlock(FireVisionDataFileBlock* block)
  : FireVisionDataFileBlock(block)
{
  _block_header = (histogram_block_header_t*) _spec_header;
  _histogram_data = (uint32_t*) _data;
}

/** Destructor. */
HistogramBlock::~HistogramBlock()
{
}

/** Returns the the width of the histogram.
 * @return the width of the histogram
 */
uint16_t
HistogramBlock::width() const
{
  return _block_header->width;
}

/** Returns the the height of the histogram.
 * @return the height of the histogram
 */
uint16_t
HistogramBlock::height() const
{
  return _block_header->height;
}

/** Returns the the depth of the histogram.
 * @return the depth of the histogram
 */
uint16_t
HistogramBlock::depth() const
{
  return _block_header->depth;
}

/** Returns the type of the object the histogram is associated with.
 * @return the object type of the histogram
 */
hint_t
HistogramBlock::object_type() const
{
  return (hint_t) _block_header->object_type;
}

/** Set the type of the object the histogram is associated with.
 * @param object_type the new type of the object
 */
void
HistogramBlock::set_object_type(hint_t object_type)
{
  _block_header->object_type = object_type;
}

/** Directly set the histogram data.
 * Note: You are reponsible that the data has the right size and format!
 * @param data pointer to the histogram data
 */
void
HistogramBlock::set_data(uint32_t* data)
{
  memcpy(_data, data, _data_size);
}

/** Store a value in a certain cell of a 2-dimensional histogram.
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param val the new value
 */
void
HistogramBlock::set_value(uint16_t x, uint16_t y, uint32_t val)
{
  if (_block_header->depth != 0)
    { throw Exception("Trying to acces a 3-dim histogram with a 2-dim access method"); }

  if (x >= _block_header->width)
    { 
      throw OutOfBoundsException("Given x value is too large (set_value, 2)", 
				 float(x), 0.0f, float(_block_header->width));
    }

  if (y >= _block_header->height)
    {
      throw OutOfBoundsException("Given y value is too large (set_value, 2)", 
				 float(y), 0.0f, float(_block_header->height));
    }

  _histogram_data[y * _block_header->width + x] = val;
}

/** Store a value in a certain cell of a 3-dimensional histogram.
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param z the z-coordinate
 * @param val the new value
 */
void
HistogramBlock::set_value(uint16_t x, uint16_t y, uint16_t z, uint32_t val)
{
  if ( x >= _block_header->width)
    {
      throw OutOfBoundsException("Given x value is too large (set_value, 3)", 
				 float(x), 0.0f, float(_block_header->width));
    }

  if ( y >= _block_header->height)
    {
      throw OutOfBoundsException("Given y value is too large (set_value, 3)", 
				 float(y), 0.0f, float(_block_header->height));
    }

  if ( z >= _block_header->depth)
    {
      throw OutOfBoundsException("Given z value is too large (set_value, 3)", 
				 float(z), 0.0f, float(_block_header->depth));
    }

  _histogram_data[z * _block_header->width * _block_header->height + y * _block_header->width + x] = val;
}

/** Obtain a certain value from a 2-dimensional histogram.
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @return the histogram value
 */
uint32_t
HistogramBlock::get_value(uint16_t x, uint16_t y)
{
  if (_block_header->depth != 0)
    { throw Exception("Trying to acces a 3-dim histogram with a 2-dim access method"); }

  if ( x >= _block_header->width)
    {
      throw OutOfBoundsException("Given x value is too large (get_value, 2)", 
				 float(x), 0.0f, float(_block_header->width));
    }

  if ( y >= _block_header->height)
    {
      throw OutOfBoundsException("Given y value is too large (get_value, 2)", 
				 float(y), 0.0f, float(_block_header->height));
    }

  return _histogram_data[y * _block_header->width + x];
}

/** Obtain a certain value from a 3-dimensional histogram.
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param z the z-coordinate
 * @return the histogram value
 */
uint32_t
HistogramBlock::get_value(uint16_t x, uint16_t y, uint16_t z)
{
  if ( x >= _block_header->width)
    {
      throw OutOfBoundsException("Given x value is too large (get_value, 3)", 
				 float(x), 0.0f, _block_header->width - 1);
    }

  if ( y >= _block_header->height)
    {
      throw OutOfBoundsException("Given y value is too large (get_value, 3)", 
				 float(y), 0.0f, _block_header->height - 1);
    }

  if ( z >= _block_header->depth)
    {
      throw OutOfBoundsException("Given z value is too large (get_value, 3)", 
				 float(z), 0.0f, _block_header->depth - 1);
    }

  return _histogram_data[z * _block_header->width * _block_header->height + y * _block_header->width + x];
}

/** Reset the histogram. */
void
HistogramBlock::reset()
{
  memset(_histogram_data, 0, _data_size);
}

} // end namespace firevision
