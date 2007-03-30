
/***************************************************************************
 *  threshold.cpp - Implementation for threshold filter, this filter will
 *                  luminance values below a given threshold to the given
 *                  min_replace value, values above a given max threshold
 *                  will be set to the max_replace value
 *
 *  Generated: Tue Jun 07 14:30:10 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include "filters/threshold.h"

#include <ippi.h>
#include <cstddef>

/** @class FilterThreshold <filters/threshold.h>
 * Threshold filter
 */

/** Constructor.
 * @param min minimum value
 * @param min_replace values below min are replaced with this value
 * @param max maximum value
 * @param max_replace values above max are replaced with this value
 */
FilterThreshold::FilterThreshold(unsigned char min, unsigned char min_replace,
				 unsigned char max, unsigned char max_replace)
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;
  this->min = min;
  this->max = max;
  this->min_replace = min_replace;
  this->max_replace = max_replace;
}


void
FilterThreshold::setSrcBuffer(unsigned char *buf, ROI *roi, orientation_t ori, unsigned int buffer_num)
{

  src = buf;
  src_roi = roi;

}

void
FilterThreshold::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}

void
FilterThreshold::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}

void
FilterThreshold::setOrientation(orientation_t ori)
{
}


/** Set new thresholds.
 * @param min minimum value
 * @param min_replace values below min are replaced with this value
 * @param max maximum value
 * @param max_replace values above max are replaced with this value
 */
void
FilterThreshold::setThresholds(unsigned char min, unsigned char min_replace,
			       unsigned char max, unsigned char max_replace)
{
  this->min = min;
  this->max = max;
  this->min_replace = min_replace;
  this->max_replace = max_replace;
}


const char *
FilterThreshold::getName()
{
  return "FilterThreshold";
}


void
FilterThreshold::apply()
{
  IppiSize size;
  size.width = src_roi->width;
  size.height = src_roi->height;

  IppStatus status;

  if ((dst == NULL) || (dst == src)) {
    // In-place
    status = ippiThreshold_GTVal_8u_C1IR( src + (src_roi->start.y * src_roi->line_step) + (src_roi->start.x * src_roi->pixel_step), src_roi->line_step,
					  size, max, max_replace );
    status = ippiThreshold_LTVal_8u_C1IR( src + (src_roi->start.y * src_roi->line_step) + (src_roi->start.x * src_roi->pixel_step), src_roi->line_step,
					  size, min, min_replace );
  } else {
    //                                base + number of bytes to line y              + pixel bytes
    status = ippiThreshold_GTVal_8u_C1R( src + (src_roi->start.y * src_roi->line_step) + (src_roi->start.x * src_roi->pixel_step), src_roi->line_step,
					 dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
					 size, max, max_replace );

    status = ippiThreshold_LTVal_8u_C1R( src + (src_roi->start.y * src_roi->line_step) + (src_roi->start.x * src_roi->pixel_step), src_roi->line_step,
					 dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
					 size, min, min_replace );
  }

  /*
  cout << "FilterThreshold: ippiFilterThreshold exit code: " << flush;
  switch (status) {
  case ippStsNoErr:
    cout << "ippStsNoErr";
    break;
  case ippStsNullPtrErr:
    cout << "ippStsNullPtrErr";
    break;
  case ippStsSizeErr:
    cout << "ippStsSizeErr";
    break;
  case ippStsStepErr:
    cout << "ippStsStepErr";
    break;
  case ippStsMaskSizeErr:
    cout << "ippStsMaskSizeErr";
    break;
  default:
    cout << "Unknown status";
  }
  cout << endl;
  */
}
