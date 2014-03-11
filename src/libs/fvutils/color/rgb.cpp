
/***************************************************************************
 *  rgb.h - RGB specific methods, macros and constants
 *
 *  Created: Sat Aug 12 14:59:55 2006
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

#include <fvutils/color/rgb.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Convert RGB to RGB with alpha values.
 * This is plain C code without special optimizations.
 * @param rgb RGB source buffer
 * @param rgb_alpha RGB with alpha destination buffer
 * @param width width in pixels
 * @param height height in pixels
 */
void
rgb_to_rgb_with_alpha_plainc(const unsigned char *rgb, unsigned char *rgb_alpha,
			     unsigned int width, unsigned int height)
{
  for ( unsigned int i = 0; i < width * height; ++i) {
    *rgb_alpha++ = *rgb++;
    *rgb_alpha++ = *rgb++;
    *rgb_alpha++ = *rgb++;
    *rgb_alpha++ = 255;
  }
}


/** Convert RGB to planar RGB.
 * This is plain C code without special optimizations.
 * @param rgb RGB source buffer
 * @param rgb_planar planar RGB buffer
 * @param width width in pixels
 * @param height height in pixels
 */
void
rgb_to_rgb_planar_plainc(const unsigned char *rgb, unsigned char *rgb_planar,
			 const unsigned int width, const unsigned int height)
{
  unsigned char *r = rgb_planar;
  unsigned char *g = rgb_planar + (width * height);
  unsigned char *b = rgb_planar + (width * height * 2);
  for ( unsigned int i = 0; i < width * height; ++i) {
    *r++ = *rgb++;
    *g++ = *rgb++;
    *b++ = *rgb++;
  }
}


/** Convert RGB to planar RGB.
 * This is plain C code without special optimizations.
 * @param rgb RGB source buffer
 * @param rgb_planar planar RGB buffer
 * @param width width in pixels
 * @param height height in pixels
 */
void
rgb_planar_to_rgb_plainc(const unsigned char *rgb_planar, unsigned char *rgb,
			 const unsigned int width, const unsigned int height)
{
  const unsigned char *r = rgb_planar;
  const unsigned char *g = rgb_planar + (width * height);
  const unsigned char *b = rgb_planar + (width * height * 2);
  for ( unsigned int i = 0; i < width * height; ++i) {
    *rgb++ = *r++;
    *rgb++ = *g++;
    *rgb++ = *b++;
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
rgb_to_bgr_with_alpha_plainc(const unsigned char *rgb, unsigned char *bgr_alpha,
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
bgr_to_rgb_plainc(const unsigned char *BGR, unsigned char *RGB,
		  unsigned int width, unsigned int height)
{
  RGB_t *rgb;
  BGR_t *bgr;
  for (unsigned int i = 0; i < (width * height); ++i) {
    bgr = (BGR_t *)BGR;
    rgb = (RGB_t *)RGB;
    rgb->R = bgr->R;
    rgb->G = bgr->G;
    rgb->B = bgr->B;
    BGR += 3;
    RGB += 3;
  }
}

/* Convert a line of a BGR buffer to a line in a planar RGB buffer, see above for general
 * notes about color space conversion from RGB to BGR
 * @param RGB where the RGB output will be written to, will have pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param BGR unsigned char array that contains the pixels, 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the YUV buffer
 * @param height Height of the image contained in the YUV buffer
 * @param rgb_line the index of the line to be converted
 * @param yuv_line the index of the line to convert to in the YUV buffer
 */

void convert_line_bgr_rgb(const unsigned char *BGR, unsigned char *RGB,
			    unsigned int width, unsigned int height)
 {
  unsigned int i = 0;
  const unsigned char *r1, *r2, *r3;
  unsigned char *n1, *n2, *n3;

  while( i < width ) {

    n1 = RGB++;
    n2 = RGB++;
    n3 = RGB++;

    r1 = BGR++;
    r2 = BGR++;
    r3 = BGR++;

    *n1 = *r3;
    *n2 = *r2;
    *n3 = *r1;


    i += 1;
  }
}

/** Convert one channel gray images  to RGB.
 * This is plain C code without special optimizations.
 * @param mono8 mono source buffer
 * @param rgb RGB destination buffer
 * @param width width in pixels
 * @param height height in pixels
 */
void
gray8_to_rgb_plainc(const unsigned char *mono8, unsigned char *rgb,
		    unsigned int width, unsigned int height)
{
  for ( unsigned int i = 0; i < width * height; ++i) {
    *rgb++ = *mono8;
    *rgb++ = *mono8;
    *rgb++ = *mono8++;
  }
}

} // end namespace firevision
