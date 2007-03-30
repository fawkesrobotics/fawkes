
/***************************************************************************
 *  invert.cpp - implementation of invert filter
 *
 *  Generated: Mon Jun 05 12:47:18 2006
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

#include <filters/invert.h>

#include <fvutils/color/yuv.h>
#include <cstddef>

/** @class FilterInvert <filters/invert.h>
 * Inversion filter.
 * This will invert the given image.
 */

/** Constructor. */
FilterInvert::FilterInvert()
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;
}


void
FilterInvert::setSrcBuffer(unsigned char *buf, ROI *roi,
			     orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterInvert::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterInvert::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}


void
FilterInvert::setOrientation(orientation_t ori)
{
}


const char *
FilterInvert::getName()
{
  return "FilterInvert";
}


void
FilterInvert::apply()
{
  if ( src == NULL ) return;
  if ( src_roi == NULL ) return;

  if ( (dst == NULL) || (dst == src) ) {
    // In-place

    register unsigned int h = 0;
    register unsigned int w = 0;

    // y-plane
    register unsigned char *yp = src + (src_roi->start.y * src_roi->line_step) + (src_roi->start.x * src_roi->pixel_step);
    
    // line starts
    unsigned char *lyp  = yp;   // y-plane

    for (h = 0; h < src_roi->height; ++h) {
      for (w = 0; w < src_roi->width; ++w) {
	*yp = 255 - *yp;
	++yp;
      }
      lyp  += src_roi->line_step;
      yp    = lyp;
    }

  } else {

    register unsigned int h = 0;
    register unsigned int w = 0;

    // y-plane
    register unsigned char *yp   = src + (src_roi->start.y * src_roi->line_step) + (src_roi->start.x * src_roi->pixel_step);
    // u-plane
    register unsigned char *up   = YUV422_PLANAR_U_PLANE(src, src_roi->image_width, src_roi->image_height)
      + ((src_roi->start.y * src_roi->line_step) / 2 + (src_roi->start.x * src_roi->pixel_step) / 2) ;
    // v-plane
    register unsigned char *vp   = YUV422_PLANAR_V_PLANE(src, src_roi->image_width, src_roi->image_height)
      + ((src_roi->start.y * src_roi->line_step) / 2 + (src_roi->start.x * src_roi->pixel_step) / 2);

    // destination y-plane
    register unsigned char *dyp  = dst + (dst_roi->start.y * dst_roi->line_step) + (dst_roi->start.x * dst_roi->pixel_step);
    // destination u-plane
    register unsigned char *dup   = YUV422_PLANAR_U_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
      + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2) ;
    // destination v-plane
    register unsigned char *dvp   = YUV422_PLANAR_V_PLANE(dst, dst_roi->image_width, dst_roi->image_height)
      + ((dst_roi->start.y * dst_roi->line_step) / 2 + (dst_roi->start.x * dst_roi->pixel_step) / 2);

    // line starts
    unsigned char *lyp  = yp;   // y-plane
    unsigned char *lup  = up;   // u-plane
    unsigned char *lvp  = vp;   // v-plane
    unsigned char *ldyp = dyp;  // destination y-plane
    unsigned char *ldup = dup;  // destination u-plane
    unsigned char *ldvp = dvp;  // destination v-plane

    for (h = 0; (h < src_roi->height) && (h < dst_roi->height); ++h) {
      for (w = 0; (w < src_roi->width) && (w < dst_roi->width); w += 2) {
	*dyp++ = 255 - *yp++;
	*dyp++ = 255 - *yp++;
	*dup++ = *up++;
	*dvp++ = *vp++;
      }

      lyp   += src_roi->line_step;
      lup   += src_roi->line_step / 2;
      lvp   += src_roi->line_step / 2;
      ldyp  += dst_roi->line_step;
      ldup  += dst_roi->line_step / 2;
      ldvp  += dst_roi->line_step / 2;
      yp     = lyp;
      up     = lup;
      vp     = lvp;
      dyp    = ldyp;
      dup    = ldup;
      dvp    = ldvp;
    }
  }

}
