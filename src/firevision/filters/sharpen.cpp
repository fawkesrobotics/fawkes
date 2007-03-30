
/***************************************************************************
 *  sharpen.cpp - Implementation of the sharpen filter
 *
 *  Generated: Thu Jun 16 16:13:15 2005
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

#include "filters/sharpen.h"

#include <ippi.h>
#include <cstddef>

/** @class FilterSharpen <filters/sharpen.h>
 * Sharpen filter.
 */

/** Constructor. */
FilterSharpen::FilterSharpen()
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;
}


void
FilterSharpen::setSrcBuffer(unsigned char *buf, ROI *roi, orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterSharpen::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterSharpen::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}


void
FilterSharpen::setOrientation(orientation_t ori)
{
}


const char *
FilterSharpen::getName()
{
  return "FilterSharpen";
}


void
FilterSharpen::apply()
{
  IppiSize size;
  size.width = src_roi->width;
  size.height = src_roi->height;

  IppStatus status;

  //                                    base + number of bytes to line y              + pixel bytes
  status = ippiFilterSharpen_8u_C1R( src + (src_roi->start.y * src_roi->line_step) + (src_roi->start.x * src_roi->pixel_step), src_roi->line_step,
				     dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
				     size );
    


  /*
  cout << "FilterSharpen: ippiFilterSharpen exit code: " << flush;
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
