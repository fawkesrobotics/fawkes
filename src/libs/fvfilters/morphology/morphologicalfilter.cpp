
/***************************************************************************
 *  morphologicalfilter.cpp - interface for a morphological filter
 *
 *  Created: Tue Mar 27 23:27:46 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvfilters/morphology/morphologicalfilter.h>

#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MorphologicalFilter <fvfilters/morphology/morphologicalfilter.h>
 * Morphological filter interface.
 * This interface defines specific API details for morphological filters.
 *
 * @author Tim Niemueller
 *
 */

/** Constructor.
 * @param name filter name
 * @param max_num_buffers maximum number of source buffers. */
MorphologicalFilter::MorphologicalFilter(const char *name, unsigned int max_num_buffers)
  : Filter(name, max_num_buffers)
{
  se = NULL;
  se_width = se_height = se_anchor_x = se_anchor_y = 0;
}


/** Destructor. */
MorphologicalFilter::~MorphologicalFilter()
{
}


/** Set the structuring element for successive filter runs.
 * @param se structuring element buffer. This is just a line-wise concatenated array
 * of values. A value of zero means ignore, any other value means to consider this
 * value.
 * @param se_width width of structuring element
 * @param se_height height of structuring element
 * @param se_anchor_x x coordinate of anchor in structuring element
 * @param se_anchor_y y coordinate of anchor in structuring element
 */
void
MorphologicalFilter::set_structuring_element(unsigned char *se,
					     unsigned int se_width, unsigned int se_height,
					     unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  this->se          = se;
  this->se_width    = se_width;
  this->se_height   = se_height;
  this->se_anchor_x = se_anchor_x;
  this->se_anchor_y = se_anchor_y;
}

} // end namespace firevision
