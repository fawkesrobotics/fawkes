
/***************************************************************************
 *  roidraw.cpp - Implementation of ROI draw filter
 *
 *  Generated: Thu Jul 14 16:01:37 2005
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

#include <filters/roidraw.h>

#include <cstddef>

/** @class FilterROIDraw <filters/roidraw.h>
 * ROI Drawing filter.
 * This filter visually marks the given region of interest.
 */

/** Constructor. */
FilterROIDraw::FilterROIDraw()
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;
}


void
FilterROIDraw::setSrcBuffer(unsigned char *buf, ROI *roi, orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterROIDraw::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterROIDraw::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
 dst = buf;
  dst_roi = roi;
}

void
FilterROIDraw::setOrientation(orientation_t ori)
{
}


const char *
FilterROIDraw::getName()
{
  return "FilterROIDraw";
}


void
FilterROIDraw::apply()
{
  // destination y-plane
  unsigned char *dyp  = dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step);
  /*
  // destination u-plane
  register unsigned char *dup  = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
                                   + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2) ;
  // destination v-plane
  register unsigned char *dvp  = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
                                   + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2);
  */

  // line starts
  unsigned char *ldyp = dyp;  // destination y-plane
  /*
  unsigned char *ldup = dup;  // destination y-plane
  unsigned char *ldvp = dvp;  // destination y-plane
  */


  // top border
  for (unsigned int i = dst_roi->start.x; i < (dst_roi->start.x + dst_roi->width); ++i) {
    *dyp = 255 - *dyp;
    dyp++;
  }

  // left and right borders
  for (unsigned int i = dst_roi->start.y; i < (dst_roi->start.y + dst_roi->height); ++i) {
    ldyp += dst_roi->line_step;
    dyp = ldyp;
    *dyp = 255 - *dyp;
    dyp += dst_roi->width;
    *dyp = 255 - *dyp;
  }
  ldyp += dst_roi->line_step;
  dyp = ldyp;

  // bottom border
  for (unsigned int i = dst_roi->start.x; i < (dst_roi->start.x + dst_roi->width); ++i) {
    *dyp = 255 - *dyp;
    dyp++;
  }

}
