
/***************************************************************************
 *  histogram_file.cpp - Histogram file
 *
 *  Created: Sat Mar 29 21:37:33 2008
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

#include <fvutils/statistical/histogram_file.h>
#include <fvutils/statistical/histogram_block.h>
#include <core/exception.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class HistogramFile <fvutils/statistical/histogram_file.h>
 * A fileformat for histograms. Such a file might contain multiple histograms, each for a
 * a different type of object.
 * @author Daniel Beck
 */

/** Constructor. */
HistogramFile::HistogramFile()
  : FireVisionDataFile(FIREVISION_HISTOGRAM_MAGIC, FIREVISION_HISTOGRAM_CURVER)
{
  attached_histograms.clear();
}


/** Destructor. */
HistogramFile::~HistogramFile()
{
  attached_histograms.clear();
}


/** Adds a new histogram block to the file.
 * @param block the histogram block
 */
void
HistogramFile::add_histogram_block(HistogramBlock* block)
{
  if ( attached_histograms.find( block->object_type() ) != attached_histograms.end() )
    { throw Exception("Cannot add another histogram of type %d to the file", block->object_type()); }

  attached_histograms[ block->object_type() ] = block;
  add_block(block);
}


/** Generates a list of histogram blocks attached to the file.
 * @return a list of all attached histogram blocks
 */
HistogramFile::HistogramBlockList
HistogramFile::histogram_blocks()
{
  FireVisionDataFile::BlockList bl = blocks();
  FireVisionDataFile::BlockList::iterator blit;

  HistogramBlockList hbl;
  
  for (blit = bl.begin(); blit != bl.end(); ++blit)
    {
      if ((*blit)->type() == FIREVISION_HISTOGRAM_TYPE_16 ||
	  (*blit)->type() == FIREVISION_HISTOGRAM_TYPE_32 )
	{
	  HistogramBlock* hb = new HistogramBlock(*blit);
	  hbl.push_back(hb);
	}
    }

  return hbl;
}


/** Get a value from a certain histogram.
 * @param object_type the requested value is obtained from the histogram for this type of
 *                    object
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param z the z-coordinate
 * @return value
 */
uint32_t
HistogramFile::get_value(hint_t object_type,
			 uint16_t x, uint16_t y, uint16_t z)
{
  if ( attached_histograms.find(object_type) == attached_histograms.end() )
    { throw Exception("File contains no histogram for type %d", object_type); }

  return attached_histograms[object_type]->get_value(x, y, z);
}


/** Set a value in a certain histogram.
 * @param object_type this specifies the type for which the respective histogram is changed
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param z the z-coordinate
 * @param val the new value for the specified cell
 */
void
HistogramFile::set_value(hint_t object_type,
			 uint16_t x, uint16_t y, uint16_t z,
			 uint32_t val)
{
  if ( attached_histograms.find(object_type) == attached_histograms.end() )
    { throw Exception("File contains no histogram for type %d", object_type); }

  attached_histograms[object_type]->set_value(x, y, z, val);
}

} // end namespace firevision
