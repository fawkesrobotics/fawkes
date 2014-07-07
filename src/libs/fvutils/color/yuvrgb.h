
/****************************************************************************
 *  yuvrgb.h - YUV to RGB conversion - specific methods, macros and constants
 *
 *  Created: Sat Aug 12 15:01:36 2006
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

#ifndef __FIREVISION_UTILS_COLOR_YUVRGB_H
#define __FIREVISION_UTILS_COLOR_YUVRGB_H

#include <fvutils/color/yuv.h>
#include <fvutils/color/rgb.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


#define YUV2RGB(y, u, v, r, g, b) {\
    r = y + ((v*1436) >> 10);			\
    g = y - ((u*352 + v*731) >> 10);		\
    b = y + ((u*1814) >> 10);			\
    r = r < 0 ? 0 : r;				\
    g = g < 0 ? 0 : g;				\
    b = b < 0 ? 0 : b;				\
    r = r > 255 ? 255 : r;			\
    g = g > 255 ? 255 : g;			\
    b = b > 255 ? 255 : b; }


#define clip(x) (unsigned char)( (x) < 0 ? 0 : ( (x) > 255 ? 255 : (x) ) )


#define yuv411packed_to_rgb(YUV, RGB, width, height) yuv411packed_to_rgb_plainc(YUV, RGB, width, height)



/** YUV to RGB Conversion
 * B = 1.164(Y - 16)                  + 2.018(U - 128)
 * G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128)
 * R = 1.164(Y - 16) + 1.596(V - 128)
 *
 * Values have to be clamped to keep them in the [0-255] range.
 * Rumour has it that the valid range is actually a subset of [0-255] (fourcc.org mentions an RGB range
 * of [16-235] mentioned) but clamping the values into [0-255] seems to produce acceptable results.
 * @param YUV unsigned char array that contains the pixels, 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param RGB where the RGB output will be written to, will have pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param width Width of the image contained in the YUV buffer
 * @param height Height of the image contained in the YUV buffer
 */
void yuv411packed_to_rgb_plainc(const unsigned char *YUV, unsigned char *RGB,
				unsigned int width, unsigned int height);


/** YUV to RGB Conversion
 * B = 1.164(Y - 16)                  + 2.018(U - 128)
 * G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128)
 * R = 1.164(Y - 16) + 1.596(V - 128)
 *
 * Values have to be clamped to keep them in the [0-255] range.
 * Rumour has it that the valid range is actually a subset of [0-255] (fourcc.org mentions an RGB range
 * of [16-235] mentioned) but clamping the values into [0-255] seems to produce acceptable results.
 * @param YUV unsigned char array that contains the pixels, 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param RGB where the RGB output will be written to, will have pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param width Width of the image contained in the YUV buffer
 * @param height Height of the image contained in the YUV buffer
 */
void yuv422planar_to_rgb_plainc(const unsigned char *planar, unsigned char *RGB,
				unsigned int width, unsigned int height);

void yuv422packed_to_rgb_plainc(const unsigned char *planar, unsigned char *RGB,
				const unsigned int width, const unsigned int height);

void yuv422planar_to_bgr_plainc(const unsigned char *planar, unsigned char *BGR,
				unsigned int width, unsigned int height);


void yuv422planar_to_rgb_with_alpha_plainc(const unsigned char *planar, unsigned char *RGB,
					   unsigned int width, unsigned int height);

void yuv422planar_to_bgr_with_alpha_plainc(const unsigned char *planar, unsigned char *BGR,
					   unsigned int width, unsigned int height);

void yuv422packed_to_bgr_with_alpha_plainc(const unsigned char *YUV, unsigned char *BGR,
					   unsigned int width, unsigned int height);


#if (defined __i386__ ||    \
     defined __386__ ||	    \
     defined __X86__ ||	    \
     defined _M_IX86 ||	    \
     defined i386 )
void yuv411planar_to_rgb_mmx (const unsigned char *yuv, unsigned char *rgb,
			      unsigned int w, unsigned int h);
#endif



inline void
pixel_yuv_to_rgb(const unsigned char y, unsigned u, unsigned char v,
		 unsigned char *r, unsigned char *g, unsigned char *b)
{
  int yt, ut, vt;

  yt = y - 16;
  ut = u - 128;
  vt = v - 128;

  *r = clip( (76284 * yt + 104595 * vt              ) >> 16 );
  *g = clip( (76284 * yt -  25625 * ut - 53281 * vt ) >> 16 );
  *b = clip( (76284 * yt + 132252 * ut              ) >> 16 );

}


/* Convert a line of a RGB buffer to a line in a planar YUV422 buffer, see above for general
 * notes about color space conversion from RGB to YUV
 * @param RGB where the RGB output will be written to, will have pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV unsigned char array that contains the pixels, 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the YUV buffer
 * @param height Height of the image contained in the YUV buffer
 * @param rgb_line the index of the line to be converted
 * @param yuv_line the index of the line to convert to in the YUV buffer
 */
inline void
convert_line_yuv422planar_to_rgb(const unsigned char *YUV, unsigned char *RGB,
				 unsigned int width, unsigned int height,
				 unsigned int yuv_line, unsigned int rgb_line)
{
  unsigned int i = 0;
  RGB_t *r1, *r2;
  const unsigned char *yp, *up, *vp;

  yp = YUV + (width * yuv_line);
  up = YUV422_PLANAR_U_PLANE(YUV, width, height) + (width * yuv_line / 2);
  vp = YUV422_PLANAR_V_PLANE(YUV, width, height) + (width * yuv_line / 2);

  RGB += 3 * width * rgb_line;

  while (i < width) {
    r1 = (RGB_t *)RGB;
    RGB += 3;
    r2 = (RGB_t *)RGB;
    RGB += 3;

    pixel_yuv_to_rgb(*yp++, *up, *vp, &(r1->R), &(r1->G), &(r1->B));
    pixel_yuv_to_rgb(*yp++, *up++, *vp++, &(r2->R), &(r2->G), &(r2->B));

    i += 2;
  }
}

} // end namespace firevision

#endif
