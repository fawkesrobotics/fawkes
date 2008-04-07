
/***************************************************************************
 *  rgb.h - RGB specific methods, macros and constants
 *
 *  Created: Sat Aug 12 14:59:55 2006
 *  based on colorspaces.h from Tue Feb 23 13:49:38 2005
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <fvutils/color/rgb.h>

/** Convert RGB to RGB with alpha values.
 * This is plain C code without special optimizations.
 * @param rgb RGB source buffer
 * @param rgb_alpha RGB with alpha destination buffer
 * @param width width in pixels
 * @param height height in pixels
 */
void
rgb_to_rgb_with_alpha_plainc(unsigned char *rgb, unsigned char *rgb_alpha,
			     unsigned int width, unsigned int height)
{
  for ( unsigned int i = 0; i < width * height; ++i) {
    *rgb_alpha++ = *rgb++;
    *rgb_alpha++ = *rgb++;
    *rgb_alpha++ = *rgb++;
    *rgb_alpha++ = 255;
  }
}


/** Convert RGB to BGR with alpha values.
 * This is plain C code without special optimizations.
 * @param rgb RGB source buffer
 * @param bgr_alpha BGR with alpha values destination buffer
 * @param width width in pixels
 * @param height height in pixels
 */
void
rgb_to_bgr_with_alpha_plainc(unsigned char *rgb, unsigned char *bgr_alpha,
			     unsigned int width, unsigned int height)
{
  for ( unsigned int i = 0; i < width * height; ++i) {
    *bgr_alpha++ = rgb[2];
    *bgr_alpha++ = rgb[1];
    *bgr_alpha++ = rgb[0];
    *bgr_alpha++ = 255;
    rgb += 3;
  }
}


/** Convert BGR to RGB
 * This is plain C code without special optimizations.
 * @param bgr BGR source buffer
 * @param rgb RGB destination buffer
 * @param width width in pixels
 * @param height height in pixels
 */
void
bgr_to_rgb_plainc(unsigned char *BGR, unsigned char *RGB,
		  unsigned int width, unsigned int height)
{
  RGB_t *rgb;
  BGR_t *bgr;
  for (register unsigned int i = 0; i < (width * height); ++i) {
    bgr = (BGR_t *)BGR;
    rgb = (RGB_t *)RGB;
    rgb->R = bgr->R;
    rgb->G = bgr->G;
    rgb->B = bgr->B;
    BGR += 3;
    RGB += 3;
  }
}

