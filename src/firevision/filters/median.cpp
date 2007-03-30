
/***************************************************************************
 *  median.cpp - Implementation of a median filter
 *
 *  Generated: Mon Jun 05 15:02:36 2006
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

#include <filters/median.h>

#include <cstddef>
#include <ippi.h>

/** @class FilterMedian <filters/median.h>
 * Median filter.
 */

/** Constructor.
 * @param mask_size size of median mask
 */
FilterMedian::FilterMedian(unsigned int mask_size)
{
  this->mask_size = mask_size;
  src = dst = NULL;
  src_roi = dst_roi = NULL;
}


void
FilterMedian::setSrcBuffer(unsigned char *buf, ROI *roi, orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterMedian::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterMedian::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}


void
FilterMedian::setOrientation(orientation_t ori)
{
}


const char *
FilterMedian::getName()
{
  return "FilterMedian";
}


void
FilterMedian::apply()
{
  IppiSize size;
  size.width = src_roi->width - mask_size;
  size.height = src_roi->height - mask_size;

  IppiSize mask = { mask_size, mask_size };
  IppiPoint anchor = { (mask_size + 1) / 2, (mask_size + 1) / 2 };

  IppStatus status;

  //                                    base + number of bytes to line y              + pixel bytes
  status = ippiFilterMedian_8u_C1R( src + ((src_roi->start.y + (mask_size + 1) / 2) * src_roi->line_step) + ((src_roi->start.x + ( mask_size + 1) / 2) * src_roi->pixel_step), src_roi->line_step,
				    dst + ((dst_roi->start.y + (mask_size + 1) / 2) * dst_roi->line_step) + ((dst_roi->start.x + ( mask_size + 1) / 2) * dst_roi->pixel_step), dst_roi->line_step,
				    size, mask, anchor );

}
