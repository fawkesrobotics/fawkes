
/***************************************************************************
 *  closing.cpp - implementation of morphological closing filter
 *
 *  Created: Mon Jun 05 14:01:15 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <filters/morphology/closing.h>

#include <filters/morphology/dilation.h>
#include <filters/morphology/erosion.h>

#include <cstddef>

/** @class FilterClosing <filters/morphology/closing.h>
 * Morphological closing.
 *
 * @author Tim Niemueller
 */


/** Constructor. */
FilterClosing::FilterClosing()
  : MorphologicalFilter("Morphological Closing")
{
  dilate = new FilterDilation();
  erode  = new FilterErosion();
}


/** Destructor. */
FilterClosing::~FilterClosing()
{
  delete dilate;
  delete erode;
}



void
FilterClosing::set_src_buffer(unsigned char *buf, ROI *roi,
			      orientation_t ori, unsigned int buffer_num)
{
  Filter::set_src_buffer(buf, roi, ori, buffer_num);
  dilate->set_src_buffer( buf, roi, ori, buffer_num );
}


void
FilterClosing::set_src_buffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  Filter::set_src_buffer(buf, roi, buffer_num);
  dilate->set_src_buffer( buf, roi, buffer_num );
}


void
FilterClosing::set_dst_buffer(unsigned char *buf, ROI *roi)
{
  Filter::set_dst_buffer(buf, roi);
  dilate->set_dst_buffer( buf, roi );
  erode->set_src_buffer( buf, roi );
}


void
FilterClosing::set_structuring_element(unsigned char *se,
				       unsigned int se_width, unsigned int se_height,
				       unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  MorphologicalFilter::set_structuring_element(se, se_width, se_height, se_anchor_x, se_anchor_y);
  dilate->set_structuring_element(se, se_width, se_height, se_anchor_x, se_anchor_y);
  erode->set_structuring_element(se, se_width, se_height, se_anchor_x, se_anchor_y);
}


void
FilterClosing::apply()
{
  dilate->apply();
  erode->apply();
}
