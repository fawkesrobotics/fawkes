
/***************************************************************************
 *  conversions.h - Conversions
 *
 *  Created: Sat Aug 12 15:19:31 2006
 *  based on colorspaces.h from Tue Feb 23 13:49:38 2005
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __FIREVISION_UTILS_COLOR_CONVERSIONS_H
#define __FIREVISION_UTILS_COLOR_CONVERSIONS_H

#include <fvutils/color/yuv.h>
#include <fvutils/color/rgb.h>
#include <fvutils/color/yuvrgb.h>
#include <fvutils/color/rgbyuv.h>
#include <fvutils/color/bayer.h>
#include <fvutils/color/colorspaces.h>

#include <core/exception.h>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Convert image from one colorspace to another.
 * This is a convenience method for unified access to all conversion routines
 * available in FireVision.
 * @param from colorspace of the src buffer
 * @param to colorspace to convert to
 * @param src source buffer
 * @param dst destination buffer
 * @param width width of image in pixels
 * @param height height of image in pixels
 * @exception Exception thrown, if the desired conversion combination is not
 * available.
 */
inline void
convert(colorspace_t   from,  colorspace_t   to,
	const unsigned char *src,   unsigned char *dst,
	unsigned int   width, unsigned int   height)
{
  if (from == to) {
    if ( src != dst ) {
      memcpy(dst, src, colorspace_buffer_size(from, width, height));
    }
  } else if ( (from == YUV422_PACKED) && (to == YUV422_PLANAR) ) {
    yuv422packed_to_yuv422planar(src, dst, width, height);
  } else if ( (from == YUY2) && (to == YUV422_PLANAR_QUARTER) ) {
    yuy2_to_yuv422planar_quarter(src, dst, width, height);
  } else if ( (from == YUY2) && (to == YUV422_PLANAR) ) {
    yuy2_to_yuv422planar(src, dst, width, height);
  } else if ( (from == YVY2) && (to == YUV422_PLANAR) ) {
    yvy2_to_yuv422planar(src, dst, width, height);

#if (			     \
     defined __i386__ ||     \
     defined __386__ ||	     \
     defined __X86__ ||	     \
     defined _M_IX86 ||	     \
     defined i386	     \
    )
  } else if ( (from == YUV411_PLANAR) && (to == RGB) ) {
    yuv411planar_to_rgb_mmx(src, dst, width, height);
#endif
  } else if ( (from == BGR) && (to == RGB) ) {
    bgr_to_rgb_plainc(src, dst, width, height);
  } else if ( (from == RGB) && (to == YUV411_PACKED) ) {
    rgb_to_yuv411packed_plainc(src, dst, width, height);
  } else if ( (from == RGB) && (to == YUV422_PLANAR) ) {
    rgb_to_yuv422planar_plainc(src, dst, width, height);
  } else if ( (from == RGB) && (to == YUV422_PACKED) ) {
    rgb_to_yuv422packed_plainc(src, dst, width, height);
  } else if ( (from == BGR) && (to == YUV422_PLANAR) ) {
    bgr_to_yuv422planar_plainc(src, dst, width, height);
  } else if ( (from == GRAY8) && (to == YUY2) ) {
    gray8_to_yuy2(src, dst, width, height);
  } else if ( (from == GRAY8) && (to == YUV422_PLANAR) ) {
    gray8_to_yuv422planar_plainc(src, dst, width, height);
  } else if ( (from == MONO8) && (to == YUV422_PLANAR) ) {
    gray8_to_yuv422planar_plainc(src, dst, width, height);
  } else if ( (from == YUV422_PLANAR) && (to == YUV422_PACKED) ) {
    yuv422planar_to_yuv422packed(src, dst, width, height);
  } else if ( (from == YUV422_PLANAR_QUARTER) && (to == YUV422_PACKED) ) {
    yuv422planar_quarter_to_yuv422packed(src, dst, width, height);
  } else if ( (from == YUV422_PLANAR_QUARTER) && (to == YUV422_PLANAR) ) {
    yuv422planar_quarter_to_yuv422planar(src, dst, width, height);
  } else if ( (from == YUV422_PLANAR) && (to == RGB) ) {
    yuv422planar_to_rgb_plainc(src, dst, width, height);
  } else if ( (from == YUV422_PACKED) && (to == RGB) ) {
    yuv422packed_to_rgb_plainc(src, dst, width, height);
  } else if ( (from == YUV422_PLANAR) && (to == BGR) ) {
    yuv422planar_to_bgr_plainc(src, dst, width, height);
  } else if ( (from == YUV422_PLANAR) && (to == RGB_WITH_ALPHA) ) {
    yuv422planar_to_rgb_with_alpha_plainc(src, dst, width, height);
  } else if ( (from == RGB) && (to == RGB_WITH_ALPHA) ) {
    rgb_to_rgb_with_alpha_plainc(src, dst, width, height);
  } else if ( (from == RGB) && (to == BGR_WITH_ALPHA) ) {
    rgb_to_bgr_with_alpha_plainc(src, dst, width, height);
  } else if ( (from == YUV422_PLANAR) && (to == BGR_WITH_ALPHA) ) {
    yuv422planar_to_bgr_with_alpha_plainc(src, dst, width, height);
  } else if ( (from == YUV422_PACKED) && (to == BGR_WITH_ALPHA) ) {
    yuv422packed_to_bgr_with_alpha_plainc(src, dst, width, height);
  } else if ( (from == BAYER_MOSAIC_GBRG) && (to == YUV422_PLANAR) ) {
    bayerGBRG_to_yuv422planar_bilinear(src, dst, width, height);
  } else if ( (from == YUV444_PACKED) && (to == YUV422_PLANAR) ) {
    yuv444packed_to_yuv422planar(src, dst, width, height);
  } else if ( (from == YUV444_PACKED) && (to == YUV422_PACKED) ) {
    yuv444packed_to_yuv422packed(src, dst, width, height);
  } else if ( (from == YVU444_PACKED) && (to == YUV422_PLANAR) ) {
    yvu444packed_to_yuv422planar(src, dst, width, height);
  } else if ( (from == YVU444_PACKED) && (to == YUV422_PACKED) ) {
    yvu444packed_to_yuv422packed(src, dst, width, height);
  } else {
    throw fawkes::Exception("Cannot convert image data from %s to %s",
			    colorspace_to_string(from),
			    colorspace_to_string(to));
  }
}


inline void
grayscale(colorspace_t cspace,
	  unsigned char *src,   unsigned char *dst,
	  unsigned int   width, unsigned int   height)
{
  switch (cspace) {
  case YUV422_PACKED:
    grayscale_yuv422packed(src, dst, width, height);
    break;
  case YUV422_PLANAR:
    grayscale_yuv422planar(src, dst, width, height);
    break;
  default:
    fawkes::Exception e("FirevisionUtils: Cannot grayscale image. "
			"Images from colorspace %s are not supported.",
			colorspace_to_string(cspace));
    throw e;
  }
}

} // end namespace firevision

#endif
