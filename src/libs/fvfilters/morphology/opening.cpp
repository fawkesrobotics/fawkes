
/***************************************************************************
 *  opening.cpp - implementation of morphological opening filter
 *
 *  Created: Mon Jun 05 14:00:46 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvfilters/morphology/opening.h>

#include <fvfilters/morphology/dilation.h>
#include <fvfilters/morphology/erosion.h>

#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterOpening <fvfilters/morphology/opening.h>
 * Morphological opening.
 *
 * @author Tim Niemueller
 */


/** Constructor. */
FilterOpening::FilterOpening()
  : MorphologicalFilter("Morphological Opening")
{
  dilate = new FilterDilation();
  erode  = new FilterErosion();
}


/** Destructor. */
FilterOpening::~FilterOpening()
{
  delete dilate;
  delete erode;
}


void
FilterOpening::set_src_buffer(unsigned char *buf, ROI *roi,
			      orientation_t ori, unsigned int buffer_num)
{
  Filter::set_src_buffer(buf, roi, ori, buffer_num);
  erode->set_src_buffer( buf, roi, ori, buffer_num );
}


void
FilterOpening::set_src_buffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  Filter::set_src_buffer(buf, roi, buffer_num);
  erode->set_src_buffer( buf, roi, buffer_num );
}


void
FilterOpening::set_dst_buffer(unsigned char *buf, ROI *roi)
{
  Filter::set_dst_buffer(buf, roi);
  erode->set_dst_buffer( buf, roi );
  dilate->set_src_buffer( buf, roi );
}


void
FilterOpening::set_structuring_element(unsigned char *se,
				       unsigned int se_width, unsigned int se_height,
				       unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  MorphologicalFilter::set_structuring_element(se, se_width, se_height, se_anchor_x, se_anchor_y);
  dilate->set_structuring_element(se, se_width, se_height, se_anchor_x, se_anchor_y);
  erode->set_structuring_element(se, se_width, se_height, se_anchor_x, se_anchor_y);
}


void
FilterOpening::apply()
{
  erode->apply();
  dilate->apply();
}

} // end namespace firevision
