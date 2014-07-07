
/****************************************************************************
 *  rgbyuv.h - RGB to YUV conversion - specific methods, macros and constants
 *
 *  Created: Sat Aug 12 15:21:39 2006
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

#include <fvutils/color/rgbyuv.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/rgb.h>
#include <fvutils/color/colorspaces.h>

#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void
rgb_to_yuy2(const unsigned char *RGB, unsigned char *YUV, unsigned int width, unsigned int height)
{
  unsigned int i, j;
  int y0, y1, u0, u1, v0, v1 ;
  int r, g, b;

  for (i = 0, j = 0; i < 3 * width * height; i += 6, j += 4) {
    r = RGB[i + 0];
    g = RGB[i + 1];
    b = RGB[i + 2];
    RGB2YUV(r, g, b, y0, u0 , v0);
    r = RGB[i + 3];
    g = RGB[i + 4];
    b = RGB[i + 5];
    RGB2YUV(r, g, b, y1, u1 , v1);
    YUV[j + 0] = y0;
    YUV[j + 1] = (u0+u1)/2;
    YUV[j + 2] = y1;
    YUV[j + 3] = (v0+v1)/2;
  }
}


/** RGB to YUV Conversion
 *
 * Y  =      (0.257 * R) + (0.504 * G) + (0.098 * B) + 16
 * Cr = V =  (0.439 * R) - (0.368 * G) - (0.071 * B) + 128
 * Cb = U = -(0.148 * R) - (0.291 * G) + (0.439 * B) + 128
 *
 * Values have to be clamped to keep them in the [0-255] range.
 * Rumour has it that the valid range is actually a subset of [0-255] (fourcc.org mentions an RGB range
 * of [16-235]) but clamping the values into [0-255] seems to produce acceptable results.
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 */
void
rgb_to_yuv411packed_plainc(const unsigned char *RGB, unsigned char *YUV,
			   unsigned int width, unsigned int height)
{
  unsigned int i = 0, j = 0;
  int y[4] = {0, 0, 0, 0}, u, v;
  RGB_t *r;
  unsigned int su = 0;
  unsigned int sv = 0;

  while (i < (width * height)) {
    r = (RGB_t *)RGB;
    for (unsigned int k = 0; j <= 4; ++j) {
      RGB2YUV(r->R, r->G, r->B, y[k], u, v);
      su += u;
      sv += v;
      RGB += 3;
    }
    YUV[j++] = su/4;
    YUV[j++] = y[0];
    YUV[j++] = y[1];
    YUV[j++] = sv/4;
    YUV[j++] = y[2];
    YUV[j++] = y[3];
  }
}


/** Convert a line of a RGB buffer to a line in a planar YUV422 buffer.
 * See above for general notes about color space conversion from RGB to YUV.
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 * @param rgb_line the index of the line to be converted
 * @param yuv_line the index of the line to convert to in the YUV buffer
 */
void
convert_line_rgb_to_yuv422planar(const unsigned char *RGB, unsigned char *YUV,
				 unsigned int width, unsigned int height,
				 unsigned int rgb_line, unsigned int yuv_line)
{
  unsigned int i = 0;
  int y1, y2, u1, u2, v1, v2;
  RGB_t *r1, *r2;
  unsigned char *yp, *up, *vp;

  yp = YUV + (width * yuv_line);
  up = YUV422_PLANAR_U_PLANE(YUV, width, height) + (width * yuv_line / 2);
  vp = YUV422_PLANAR_V_PLANE(YUV, width, height) + (width * yuv_line / 2);

  RGB += 3 * width * rgb_line;

  while (i < width) {
    r1 = (RGB_t *)RGB;
    RGB += 3;
    r2 = (RGB_t *)RGB;
    RGB += 3;

    RGB2YUV(r1->R, r1->G, r1->B, y1, u1, v1);
    RGB2YUV(r2->R, r2->G, r2->B, y2, u2, v2);

    *yp++ = y1;
    *yp++ = y2;
    *up++ = (u1 + u2) / 2;
    *vp++ = (v1 + v2) / 2;

    i += 2;
  }
}


/** Convert an RGB buffer to a planar YUV422 buffer.
 * See above for general notes about color space conversion from RGB to YUV.
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 */
void
rgb_to_yuv422planar_plainc(const unsigned char *RGB, unsigned char *YUV,
			   unsigned int width, unsigned int height)
{
  unsigned int i = 0;
  int y1, y2, u1, u2, v1, v2;
  RGB_t *r1, *r2;
  unsigned char *yp, *up, *vp;

  yp = YUV;
  up = YUV422_PLANAR_U_PLANE(YUV, width, height);
  vp = YUV422_PLANAR_V_PLANE(YUV, width, height);

  while (i < (width * height)) {
    r1 = (RGB_t *)RGB;
    RGB += 3;
    r2 = (RGB_t *)RGB;
    RGB += 3;

    RGB2YUV(r1->R, r1->G, r1->B, y1, u1, v1);
    RGB2YUV(r2->R, r2->G, r2->B, y2, u2, v2);

    *yp++ = y1;
    *yp++ = y2;
    *up++ = (u1 + u2) / 2;
    *vp++ = (v1 + v2) / 2;

    i += 2;
  }
}

/* Convert a line of a RGB buffer to a line in a packed YUV422 buffer, see above for general
 * notes about color space conversion from RGB to YUV
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 * @param rgb_line the index of the line to be converted
 * @param yuv_line the index of the line to convert to in the YUV buffer
 */
void
convert_line_rgb_to_yuv422packed(const unsigned char *RGB, unsigned char *YUV,
				 unsigned int width, unsigned int height,
				 unsigned int rgb_line, unsigned int yuv_line)
{
  unsigned int i = 0;
  int y1, y2, u1, u2, v1, v2;
  RGB_t *r1, *r2;
  unsigned char *p;

  p = YUV + (width * yuv_line) * 2;

  RGB += 3 * width * rgb_line;

  while (i < width) {
    r1 = (RGB_t *)RGB;
    RGB += 3;
    r2 = (RGB_t *)RGB;
    RGB += 3;

    RGB2YUV(r1->R, r1->G, r1->B, y1, u1, v1);
    RGB2YUV(r2->R, r2->G, r2->B, y2, u2, v2);

    *p++ = (u1 + u2) / 2;
    *p++ = y1;
    *p++ = (v1 + v2) / 2;
    *p++ = y2;

    i += 2;
  }
}


/* Convert an RGB buffer to a packed YUV422 buffer, see above for general notes about color space
 * conversion from RGB to YUV
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 */
void
rgb_to_yuv422packed_plainc(const unsigned char *RGB, unsigned char *YUV,
			   unsigned int width, unsigned int height)
{
  unsigned int i = 0;
  int y1, y2, u1, u2, v1, v2;
  RGB_t *r1, *r2;
  unsigned char *p;

  p = YUV;

  while (i < (width * height)) {
    r1 = (RGB_t *)RGB;
    RGB += 3;
    r2 = (RGB_t *)RGB;
    RGB += 3;

    RGB2YUV(r1->R, r1->G, r1->B, y1, u1, v1);
    RGB2YUV(r2->R, r2->G, r2->B, y2, u2, v2);

    *p++ = (u1 + u2) / 2;
    *p++ = y1;
    *p++ = (v1 + v2) / 2;
    *p++ = y2;

    i += 2;
  }
}


void
rgb_planar_to_yuv422packed_plainc(const unsigned char *rgb_planar, unsigned char *YUV,
				  unsigned int width, unsigned int height)
{
  const unsigned char *r = rgb_planar;
  const unsigned char *g = rgb_planar + (width * height);
  const unsigned char *b = rgb_planar + (width * height * 2);

  unsigned int i = 0;
  int y1, y2, u1, u2, v1, v2;
  unsigned char *p;
  unsigned char r1, r2, g1, g2, b1, b2;

  p = YUV;

  while (i < (width * height)) {

    r1 = *r++;
    r2 = *r++;
    g1 = *g++;
    g2 = *g++;
    b1 = *b++;
    b2 = *b++;
    RGB2YUV(r1, g1, b1, y1, u1, v1);
    RGB2YUV(r2, g2, b2, y2, u2, v2);

    *p++ = (u1 + u2) / 2;
    *p++ = y1;
    *p++ = (v1 + v2) / 2;
    *p++ = y2;

    i += 2;
  }
}

/** Convert an BGR buffer to a planar YUV422 buffer.
 * See above for general notes about color space conversion from RGB to YUV.
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 */
void
bgr_to_yuv422planar_plainc(const unsigned char *BGR, unsigned char *YUV,
			   unsigned int width, unsigned int height)
{
  unsigned int i = 0;
  int y1, y2, u1, u2, v1, v2;
  BGR_t *r1, *r2;
  unsigned char *yp, *up, *vp;

  yp = YUV;
  up = YUV422_PLANAR_U_PLANE(YUV, width, height);
  vp = YUV422_PLANAR_V_PLANE(YUV, width, height);

  while (i < (width * height)) {
    r1 = (BGR_t *)BGR;
    BGR += 3;
    r2 = (BGR_t *)BGR;
    BGR += 3;

    RGB2YUV(r1->R, r1->G, r1->B, y1, u1, v1);
    RGB2YUV(r2->R, r2->G, r2->B, y2, u2, v2);

    *yp++ = y1;
    *yp++ = y2;
    *up++ = (u1 + u2) / 2;
    *vp++ = (v1 + v2) / 2;

    i += 2;
  }
}

} // end namespace firevision
