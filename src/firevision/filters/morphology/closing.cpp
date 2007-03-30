
/***************************************************************************
 *  closing.cpp - implementation of morphological closing filter
 *
 *  Generated: Mon Jun 05 14:01:15 2006
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
{
  dilate = new FilterDilation();
  erode  = new FilterErosion();

  src = dst = NULL;
  src_roi = dst_roi = NULL;
}


/** Destructor. */
FilterClosing::~FilterClosing()
{
  delete dilate;
  delete erode;
}


void
FilterClosing::setSrcBuffer(unsigned char *buf, ROI *roi,
			     orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;

  dilate->setSrcBuffer( buf, roi, ori, buffer_num );
}


void
FilterClosing::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;

  dilate->setSrcBuffer( src, src_roi, buffer_num );
} 


void
FilterClosing::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;

  dilate->setDstBuffer( dst, dst_roi, ori );
  erode->setSrcBuffer( dst, dst_roi, ori );
}


void
FilterClosing::setStructuringElement(unsigned char *se,
				     unsigned int se_width, unsigned int se_height,
				     unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  dilate->setStructuringElement(se, se_width, se_height, se_anchor_x, se_anchor_y);
  erode->setStructuringElement(se, se_width, se_height, se_anchor_x, se_anchor_y);
}


void
FilterClosing::setOrientation(orientation_t ori)
{
}


const char *
FilterClosing::getName()
{
  return "FilterClosing";
}


void
FilterClosing::apply()
{
  dilate->apply();
  erode->apply();
}
