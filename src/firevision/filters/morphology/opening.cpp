
/***************************************************************************
 *  opening.cpp - implementation of morphological opening filter
 *
 *  Generated: Mon Jun 05 14:00:46 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <filters/morphology/opening.h>

#include <filters/morphology/dilation.h>
#include <filters/morphology/erosion.h>

#include <cstddef>

/** @class FilterOpening <filters/morphology/opening.h>
 * Morphological opening.
 *
 * @author Tim Niemueller
 */


/** Constructor. */
FilterOpening::FilterOpening()
{
  dilate = new FilterDilation();
  erode  = new FilterErosion();

  src = dst = NULL;
  src_roi = dst_roi = NULL;
}


/** Destructor. */
FilterOpening::~FilterOpening()
{
  delete dilate;
  delete erode;
}


void
FilterOpening::setSrcBuffer(unsigned char *buf, ROI *roi,
			     orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;

  erode->setSrcBuffer( buf, roi, ori, buffer_num );
}


void
FilterOpening::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;

  erode->setSrcBuffer( buf, roi, buffer_num );
}


void
FilterOpening::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;

  erode->setDstBuffer( buf, roi, ori );
  dilate->setSrcBuffer( buf, roi, ori );
}


void
FilterOpening::setOrientation(orientation_t ori)
{
}


void
FilterOpening::setStructuringElement(unsigned char *se,
				     unsigned int se_width, unsigned int se_height,
				     unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  dilate->setStructuringElement(se, se_width, se_height, se_anchor_x, se_anchor_y);
  erode->setStructuringElement(se, se_width, se_height, se_anchor_x, se_anchor_y);
}


const char *
FilterOpening::getName()
{
  return "FilterOpening";
}


void
FilterOpening::apply()
{
  erode->apply();
  dilate->apply();
}
