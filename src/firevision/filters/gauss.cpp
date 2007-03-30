
/***************************************************************************
 *  gauss.cpp - Implementation of a Gauss filter
 *
 *  Generated: Thu May 12 09:33:55 2005
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

#include "filters/gauss.h"

#include <cstddef>
#include <ippi.h>

/** @class FilterGauss <filters/gauss.h>
 * Gaussian filter.
 * Applies Gaussian linear filter to image (blur effect).
 */

/** Constructor. */
FilterGauss::FilterGauss()
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;
}


void
FilterGauss::setSrcBuffer(unsigned char *buf, ROI *roi, orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterGauss::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}

void
FilterGauss::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}

void
FilterGauss::setOrientation(orientation_t ori)
{
}


const char *
FilterGauss::getName()
{
  return "FilterGauss";
}


void
FilterGauss::apply()
{
  IppiSize size;
  size.width = src_roi->width;
  size.height = src_roi->height;

  /* IppStatus status = */ ippiFilterGauss_8u_C1R( src + (src_roi->start.y * src_roi->line_step) + (src_roi->start.x * src_roi->pixel_step), src_roi->line_step,
					     dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step), dst_roi->line_step,
					     size,
					     ippMskSize5x5 );

  /*
  cout << "FilterGauss: ippiFilterGauss exit code: " << flush;
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
