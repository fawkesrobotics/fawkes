
/***************************************************************************
 *  min.cpp - implementation of min intensity filter
 *
 *  Generated: Mon Jun 05 16:57:57 2006
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

#include <filters/min.h>

#include <fvutils/color/yuv.h>
#include <cstddef>


/** @class FilterMin <filters/min.h>
 * Minimum filter
 */

/** Constructor. */
FilterMin::FilterMin()
{
  src[0] = src[1] = dst = NULL;
  src_roi[0] = src_roi[1] = dst_roi = NULL;
}


void
FilterMin::setSrcBuffer(unsigned char *buf, ROI *roi,
			     orientation_t ori, unsigned int buffer_num)
{
  if ( buffer_num < 2 ) {
    src[buffer_num] = buf;
    src_roi[buffer_num] = roi;
  }
}


void
FilterMin::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  if ( buffer_num < 2 ) {
    src[buffer_num] = buf;
    src_roi[buffer_num] = roi;
  }
}


void
FilterMin::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}


void
FilterMin::setOrientation(orientation_t ori)
{
}


const char *
FilterMin::getName()
{
  return "FilterMin";
}


void
FilterMin::apply()
{
  if ( src[0] == NULL ) return;
  if ( src[1] == NULL ) return;
  if ( src_roi[0] == NULL ) return;
  if ( src_roi[1] == NULL ) return;

  register unsigned int h = 0;
  register unsigned int w = 0;


  // y-plane
  register unsigned char *byp   = src[0] + (src_roi[0]->start.y * src_roi[0]->line_step) + (src_roi[0]->start.x * src_roi[0]->pixel_step);
  // u-plane
  register unsigned char *bup   = YUV422_PLANAR_U_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
    + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2) ;
  // v-plane
  register unsigned char *bvp   = YUV422_PLANAR_V_PLANE(src[0], src_roi[0]->image_width, src_roi[0]->image_height)
    + ((src_roi[0]->start.y * src_roi[0]->line_step) / 2 + (src_roi[0]->start.x * src_roi[0]->pixel_step) / 2);


  // y-plane
  register unsigned char *fyp   = src[1] + (src_roi[1]->start.y * src_roi[1]->line_step) + (src_roi[1]->start.x * src_roi[1]->pixel_step);
  // u-plane
  register unsigned char *fup   = YUV422_PLANAR_U_PLANE(src[1], src_roi[1]->image_width, src_roi[1]->image_height)
    + ((src_roi[1]->start.y * src_roi[1]->line_step) / 2 + (src_roi[1]->start.x * src_roi[1]->pixel_step) / 2) ;
  // v-plane
  register unsigned char *fvp   = YUV422_PLANAR_V_PLANE(src[1], src_roi[1]->image_width, src_roi[1]->image_height)
    + ((src_roi[1]->start.y * src_roi[1]->line_step) / 2 + (src_roi[1]->start.x * src_roi[1]->pixel_step) / 2);


  // destination y-plane
  register unsigned char *dyp  = dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step);
  // destination u-plane
  register unsigned char *dup   = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
    + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2) ;
  // destination v-plane
  register unsigned char *dvp   = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
    + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2);
  
  // line starts
  unsigned char *lbyp = byp;   // y-plane
  unsigned char *lbup = fup;   // u-plane
  unsigned char *lbvp = fvp;   // v-plane
  unsigned char *lfyp = fyp;   // y-plane
  unsigned char *lfup = fup;   // u-plane
  unsigned char *lfvp = fvp;   // v-plane
  unsigned char *ldyp = dyp;  // destination y-plane
  unsigned char *ldup = dup;  // destination u-plane
  unsigned char *ldvp = dvp;  // destination v-plane

  unsigned char u1, u2, v1, v2;

  for (h = 0; (h < src_roi[1]->height) && (h < dst_roi->height); ++h) {
    for (w = 0; (w < src_roi[1]->width) && (w < dst_roi->width); w += 2) {
      if ( *byp < *fyp ) {
	*dyp++ = *byp;
	u1 = *bup;
	v1 = *bvp;
      } else {
	*dyp++ = *fyp;
	u1 = *fup;
	v1 = *fvp;
      }
      ++byp;
      ++fyp;

      if ( *byp < *fyp ) {
	*dyp++ = *byp;
	u2 = *bup;
	v2 = *bvp;
      } else {
	*dyp++ = *fyp;
	u2 = *fup;
	v2 = *fvp;
      }
      ++byp;
      ++fyp;

      *dup++ = (u1 + u2) / 2;
      *dvp++ = (v1 + v2) / 2;

      ++bup;
      ++bvp;
      ++fup;
      ++fvp;
    }

    lbyp   += src_roi[0]->line_step;
    lbup   += src_roi[0]->line_step / 2;
    lbvp   += src_roi[0]->line_step / 2;
    lfyp   += src_roi[1]->line_step;
    lfup   += src_roi[1]->line_step / 2;
    lfvp   += src_roi[1]->line_step / 2;
    ldyp  += dst_roi->line_step;
    ldup  += dst_roi->line_step / 2;
    ldvp  += dst_roi->line_step / 2;
    byp    = lbyp;
    bup    = lbup;
    bvp    = lbvp;
    fyp    = lfyp;
    fup    = lfup;
    fvp    = lfvp;
    dyp    = ldyp;
    dup    = ldup;
    dvp    = ldvp;
  }

}
