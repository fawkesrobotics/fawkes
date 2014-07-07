
/***************************************************************************
 *  bayer.cpp - methods to convert bayer mosaic images to other formats
 *
 *  Generated: Fri Aug 11 00:03:32 2006
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

#include <fvutils/color/bayer.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/rgbyuv.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/* The basic information has been taken from
 * http://www-ise.stanford.edu/~tingchen/
 */

#define assign(y, u, v, y1, u1, v1, y2, u2, v2) { \
    *y++ = y1;					  \
    *y++ = y2;					  \
    *u++ = ((u1 + u2) >> 1);			  \
    *v++ = ((v1 + v2) >> 1); }


void
bayerGBRG_to_yuv422planar_nearest_neighbour(const unsigned char *bayer, unsigned char *yuv,
					    unsigned int width, unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  const unsigned char *b = bayer;

  int y1, u1, v1, y2, u2, v2;
  int t1, t2;

  for ( unsigned int h = 0; h < height; h += 2) {

    // g  b  ... line
    for (unsigned int w = 0; w < width; w += 2) {
      t1 = b[width];
      t2 = b[1];
      RGB2YUV(t1, *b, t2, y1, u1, v1);
      ++b;

      t1 = b[width - 1];
      t2 = b[-1];
      RGB2YUV(t1, t2, *b, y2, u2, v2);
      ++b;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    // r  g  ... line
    for (unsigned int w = 0; w < width; w += 2) {
      t1 = b[1];
      t2 = b[-width+1];
      RGB2YUV(*b, t1, t2, y1, u1, v1);
      ++b;

      t1 = b[-1];
      t2 = b[-width];
      RGB2YUV(t1, *b, t2, y2, u2, v2);
      ++b;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }
  }
}



void
bayerGBRG_to_yuv422planar_bilinear(const unsigned char *bayer, unsigned char *yuv,
				   unsigned int width, unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  const unsigned char *bf = bayer;

  int y1, u1, v1, y2, u2, v2;
  int r, g, b;

  // first line is special
  // g  b  ... line
  // not full data in first columns
  RGB2YUV(bf[width], *bf, bf[1], y1, u1, v1);
  ++bf;

  r = (bf[width - 1] + bf[width + 1]) >> 1;
  // correct:
  // g = (bf[-1] + bf[width] + bf[1]) / 3;
  // faster:
  g = (bf[-1] + bf[1]) >> 1;
  RGB2YUV(r, g, *bf, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  // rest of first line
  for (unsigned int w = 2; w < width - 2; w += 2) {
    b = (bf[-1] + bf[1]) >> 1;
    RGB2YUV(bf[width], *bf, b, y1, u1, v1);
    ++bf;

    r = (bf[width - 1] + bf[width + 1]) >> 1;
    // correct:
    // g = (bf[-1] + bf[width] + bf[1]) / 3;
    // faster:
    g = (bf[-1] + bf[1]) >> 1;
    RGB2YUV(r, g, *bf, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // not full data in last columns
  b = (bf[-1] + bf[1]) >> 1;
  RGB2YUV(bf[width], *bf, b, y1, u1, v1);
  ++bf;

  g = (bf[-1] + bf[width]) >> 1;
  RGB2YUV(bf[width - 1], g, *bf, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  for ( unsigned int h = 1; h < height - 1; h += 2) {

    // r  g  ... line
    // correct: g = (bf[-width] + bf[1] + bf[width]) / 3;
    // faster:
    g = (bf[-width] + bf[1]) >> 1;
    b = (bf[width-1] + bf[width+1]) >> 1;
    RGB2YUV(*bf, g, b, y1, u1, v1);
    ++bf;

    r = (bf[-1] + bf[1]) >> 1;
    b = (bf[-width] + bf[width]) >> 1;
    RGB2YUV(r, *bf, b, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);

    for (unsigned int w = 2; w < width - 2; w += 2) {
      g = (bf[-width] + bf[1] + bf[width] + bf[-1]) >> 2;
      b = (bf[-width-1] + bf[-width+1] + bf[width-1] + bf[width+1]) >> 2;
      RGB2YUV(*bf, g, b, y1, u1, v1);
      ++bf;

      r = (bf[-1] + bf[1]) >> 1;
      b = (bf[-width] + bf[width]) >> 1;
      RGB2YUV(r, *bf, b, y2, u2, v2);
      ++bf;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    g = (bf[-width] + bf[1] + bf[width] + bf[-1]) >> 2;
    b = (bf[-width-1] + bf[-width+1] + bf[width-1] + bf[width+1]) >> 2;
    RGB2YUV(*bf, g, b, y1, u1, v1);
    ++bf;

    b = (bf[-width] + bf[width]) >> 1;
    RGB2YUV(bf[-1], *bf, g, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);



    // g  b  ... line
    r = (bf[width] + bf[-width]) >> 1;
    RGB2YUV(r, *bf, bf[1], y1, u1, v1);
    ++bf;

    r = (bf[-width-1] + bf[-width+1] + bf[width - 1] + bf[width + 1]) >> 2;
    g = (bf[-width] + bf[1] + bf[width] + bf[-1]) >> 2;
    RGB2YUV(r, g, *bf, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);

    for (unsigned int w = 2; w < width - 2; w += 2) {
      r = (bf[width] + bf[-width]) >> 1;
      b = (bf[-1] + bf[1]) >> 1;
      RGB2YUV(r, *bf, b, y1, u1, v1);
      ++bf;

      r = (bf[-width-1] + bf[-width+1] + bf[width-1] + bf[width+1]) >> 2;
      g = (bf[-width] + bf[1] + bf[width] + bf[-1]) >> 2;
      RGB2YUV(r, g, *bf, y2, u2, v2);
      ++bf;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    r = (bf[width] + bf[-width]) >> 1;
    b = (bf[-1] + bf[1]) >> 1;
    RGB2YUV(r, *bf, b, y1, u1, v1);
    ++bf;

    r = (bf[-width-1] + bf[width-1]) >> 1;
    // correct: g = (bf[-width] + bf[width] + bf[-1]) / 3;
    // faster:
    g = (bf[-width] + bf[-1]) >> 1;
    RGB2YUV(r, g, *bf, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // last r  g  ... line
  // correct: g = (bf[-width] + bf[1] + bf[width]) / 3;
  // faster:
  g = (bf[-width] + bf[1]) >> 1;
  b = bf[-width+1];
  RGB2YUV(*bf, g, b, y1, u1, v1);
  ++bf;

  r = (bf[-1] + bf[1]) >> 1;
  b = bf[-width];
  RGB2YUV(r, g, *bf, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  for (unsigned int w = 2; w < width - 2; w += 2) {
    // correct: g = (bf[-width] + bf[1] + bf[-1]) / 3
    // faster:
    g = (bf[-width] + bf[-1]) >> 1;
    b = (bf[-width-1] + bf[-width+1]) >> 1;
    RGB2YUV(*bf, g, b, y1, u1, v1);
    ++bf;

    r = (bf[-1] + bf[1]) >> 1;
    b = bf[-width];
    RGB2YUV(r, *bf, b, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // correct: g = (bf[-width] + bf[1] + bf[-1]) / 3;
  // faster:
  g = (bf[-width] + bf[-1]) >> 1;
  b = (bf[-width-1] + bf[-width+1]) >> 1;
  RGB2YUV(*bf, g, b, y1, u1, v1);
  ++bf;

  b = bf[-width];
  RGB2YUV(bf[-1], *bf, b, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

}


void
bayerGBRG_to_yuv422planar_bilinear2(const unsigned char *bayer, unsigned char *yuv,
				   unsigned int width, unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  const unsigned char *bf = bayer;

  int y1, u1, v1, y2, u2, v2;
  int r, g, b;

  // ignore first g  b  ... line
  bf += width;
  y  += width;
  u  += width >> 1;
  v  += width >> 1;

  for ( unsigned int h = 1; h < height - 1; h += 2) {

    // r  g  ... line
    // ignore first two columns
    ++bf; ++bf;
    ++y; ++y;
    ++u; ++v;

    for (unsigned int w = 2; w < width - 2; w += 2) {
      g = (bf[1] + bf[-1]) >> 1;
      b = (bf[width-1] + bf[width+1]) >> 1;
      RGB2YUV(*bf, g, b, y1, u1, v1);
      ++bf;

      r = (bf[-1] + bf[1]) >> 1;
      b = (bf[-width] + bf[width]) >> 1;
      RGB2YUV(r, *bf, b, y2, u2, v2);
      ++bf;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    // ignore last two columns
    ++bf; ++bf;
    ++y; ++y;
    ++u; ++v;

    // g  b  ... line
    // ignore first two columns
    ++bf; ++bf;
    ++y; ++y;
    ++u; ++v;

    for (unsigned int w = 2; w < width - 2; w += 2) {
      r = (bf[width] + bf[-width]) >> 1;
      b = (bf[-1] + bf[1]) >> 1;
      RGB2YUV(r, *bf, b, y1, u1, v1);
      ++bf;

      r = (bf[width-1] + bf[width+1]) >> 1;
      g = (bf[1] + bf[width]) >> 1;
      RGB2YUV(r, g, *bf, y2, u2, v2);
      ++bf;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    // ignore last two columns
    ++bf; ++bf;
    ++y; ++y;
    ++u; ++v;
  }

  // ignore last r  g  ... line

}

/* Not faster in benchmarks
void
bayerGBRG_to_yuv422planar_bilinear2(const unsigned char *bayer, unsigned char *yuv,
				    unsigned int width, unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  unsigned char *bf = bayer;

  int y1, u1, v1, y2, u2, v2;
  int r, g, b;

  // first line is special
  // g  b  ... line
  // not full data in first columns
  RGB2YUV(*(bf + width), *bf, *(bf + 1), y1, u1, v1);
  ++bf;

  r = (*(bf + width - 1) + *(bf + width + 1)) >> 1;
  // correct:
  // g = (*(bf - 1) + *(bf + width) + *(bf + 1)) / 3;
  // faster:
  g = (*(bf -1) + *(bf +1)) >> 1;
  RGB2YUV(r, g, *bf, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  // rest of first line
  for (unsigned int w = 2; w < width - 2; w += 2) {
    b = (*(bf - 1) + *(bf + 1)) >> 1;
    RGB2YUV(*(bf + width), *bf, b, y1, u1, v1);
    ++bf;

    r = (*(bf + width - 1) + *(bf + width + 1)) >> 1;
    // correct:
    // g = (*(bf - 1) + *(bf + width) + *(bf + 1)) / 3;
    // faster:
    g = (*(bf - 1) + *(bf + 1)) >> 1;
    RGB2YUV(r, g, *bf, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // not full data in last columns
  b = (*(bf - 1) + *(bf + 1)) >> 1;
  RGB2YUV(*(bf + width), *bf, b, y1, u1, v1);
  ++bf;

  g = (*(bf - 1) + *(bf + width)) >> 1;
  RGB2YUV(*(bf + width - 1), g, *bf, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  for ( unsigned int h = 1; h < height - 1; h += 2) {

    // r  g  ... line
    // correct: g = (*(bf - width) + *(bf + 1) + *(bf + width)) / 3;
    // faster:
    g = (*(bf - width) + *(bf + 1)) >> 1;
    b = (*(bf + width - 1) + *(bf + width + 1)) >> 1;
    RGB2YUV(*bf, g, b, y1, u1, v1);
    ++bf;

    r = (*(bf - 1) + *(bf + 1)) >> 1;
    b = (*(bf - width) + *(bf + width)) >> 1;
    RGB2YUV(r, *bf, g, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);

    for (unsigned int w = 2; w < width - 2; w += 2) {
      g = (*(bf - width) + *(bf + 1) + *(bf + width) + *(bf - 1)) >> 2;
      b = (*(bf - width - 1) + *(bf - width + 1) + *(bf + width - 1) + *(bf + width + 1)) >> 2;
      RGB2YUV(*bf, g, b, y1, u1, v1);
      ++bf;

      r = (*(bf - 1) + *(bf + 1)) >> 1;
      b = (*(bf - width) + *(bf + width)) >> 1;
      RGB2YUV(r, *bf, b, y2, u2, v2);
      ++bf;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    g = (*(bf - width) + *(bf + 1) + *(bf + width) + *(bf - 1)) >> 2;
    b = (*(bf - width - 1) + *(bf - width + 1) + *(bf + width - 1) + *(bf + width + 1)) >> 2;
    RGB2YUV(*bf, g, b, y1, u1, v1);
    ++bf;

    b = (*(bf - width) + *(bf + width)) >> 1;
    RGB2YUV(*(bf - 1), *bf, g, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);



    // g  b  ... line
    r = (*(bf + width) + *(bf - width)) >> 1;
    RGB2YUV(r, *bf, *(bf + 1), y1, u1, v1);
    ++bf;

    r = (*(bf - width - 1) + *(bf - width + 1) + *(bf + width - 1) + *(bf + width + 1)) >> 2;
    g = (*(bf - width) + *(bf + 1) + *(bf + width) + *(bf - 1)) >> 2;
    RGB2YUV(r, g, *bf, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);

    for (unsigned int w = 2; w < width - 2; w += 2) {
      r = (*(bf + width) + *(bf - width)) >> 1;
      b = (*(bf - 1) + *(bf + 1)) >> 1;
      RGB2YUV(r, *bf, b, y1, u1, v1);
      ++bf;

      r = (*(bf - width - 1) + *(bf - width + 1) + *(bf + width - 1) + *(bf + width + 1)) >> 2;
      g = (*(bf - width) + *(bf + 1) + *(bf + width) + *(bf - 1)) >> 2;
      RGB2YUV(r, g, *bf, y2, u2, v2);
      ++bf;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    r = (*(bf + width) + *(bf - width)) >> 1;
    b = (*(bf - 1) + *(bf + 1)) >> 1;
    RGB2YUV(r, *bf, b, y1, u1, v1);
    ++bf;

    r = (*(bf - width - 1) + *(bf + width - 1)) >> 1;
    // correct: g = (*(bf - width) + *(bf + width) + *(bf - 1)) / 3;
    // faster:
    g = (*(bf - width) + *(bf - 1)) >> 1;
    RGB2YUV(r, g, *bf, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // last r  g  ... line
  // correct: g = (*(bf - width) + *(bf + 1) + *(bf + width)) / 3;
  // faster:
  g = (*(bf - width) + *(bf + 1)) >> 1;
  b = *(bf - width + 1);
  RGB2YUV(*bf, g, b, y1, u1, v1);
  ++bf;

  r = (*(bf - 1) + *(bf + 1)) >> 1;
  b = *(bf - width);
  RGB2YUV(r, g, *bf, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  for (unsigned int w = 2; w < width - 2; w += 2) {
    // correct: g = (*(bf - width) + *(bf + 1) + *(bf - 1)) / 3
    // faster:
    g = (*(bf - width) + *(bf - 1)) >> 1;
    b = (*(bf - width - 1) + *(bf - width + 1)) >> 1;
    RGB2YUV(*bf, g, b, y1, u1, v1);
    ++bf;

    r = (*(bf - 1) + *(bf + 1)) >> 1;
    b = *(bf - width);
    RGB2YUV(r, *bf, b, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // correct: g = (*(bf - width) + *(bf + 1) + *(bf - 1)) / 3;
  // faster:
  g = (*(bf - width) + *(bf - 1)) >> 1;
  b = (*(bf - width - 1) + *(bf - width + 1)) >> 1;
  RGB2YUV(*bf, g, b, y1, u1, v1);
  ++bf;

  b = *(bf - width);
  RGB2YUV(*(bf - 1), *bf, b, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

}
*/

void
bayerGRBG_to_yuv422planar_nearest_neighbour(const unsigned char *bayer,
					    unsigned char *yuv,
					    unsigned int width,
					    unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  const unsigned char *b = bayer;

  int y1, u1, v1, y2, u2, v2;

  for ( unsigned int h = 0; h < height; h += 2) {

    // g  r  ... line
    for (unsigned int w = 0; w < width; w += 2) {
      RGB2YUV(b[1], b[width], *b, y1, u1, v1);
      ++b;

      RGB2YUV(*b, b[-1], b[width - 1], y2, u2, v2);
      ++b;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    // b  g  ... line
    for (unsigned int w = 0; w < width; w += 2) {
      RGB2YUV(*(b-width+1), b[1], *b, y1, u1, v1);
      ++b;

      RGB2YUV(*(b-width), *b, b[-1], y2, u2, v2);
      ++b;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }
  }
}


void
bayerRGGB_to_yuv422planar_nearest_neighbour(const unsigned char *bayer,
					    unsigned char *yuv,
					    unsigned int width,
					    unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  const unsigned char *b = bayer;

  int y1, u1, v1, y2, u2, v2;

  for ( unsigned int h = 0; h < height; h += 2) {

    // r  g  ... line
    for (unsigned int w = 0; w < width; w += 2) {
      RGB2YUV(*b, b[1], b[width+1], y1, u1, v1);
      ++b;

      RGB2YUV(b[-1], *b, b[width], y2, u2, v2);
      ++b;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    // g  b  ... line
    for (unsigned int w = 0; w < width; w += 2) {
      RGB2YUV(*(b-width), *b, b[1], y1, u1, v1);
      ++b;

      RGB2YUV(*(b-width-1), b[-1], *b, y2, u2, v2);
      ++b;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }
  }
}

void
bayerGRBG_to_yuv422planar_bilinear(const unsigned char *bayer, unsigned char *yuv,
				   unsigned int width, unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  const unsigned char *bf = bayer;

  int y1, u1, v1, y2, u2, v2;
  int r, g, b;

  // first line is special
  // g  r  ... line
  // not full data in first columns
  RGB2YUV(bf[1], *bf, bf[width], y1, u1, v1);
  ++bf;

  b = (bf[width - 1] + bf[width + 1]) >> 1;
  // correct:
  // g = (bf[-1] + bf[width] + bf[1]) / 3;
  // faster:
  g = (bf[-1] + bf[1]) >> 1;
  RGB2YUV(*bf, g, b, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  // rest of first line
  for (unsigned int w = 2; w < width - 2; w += 2) {
    r = (bf[-1] + bf[1]) >> 1;
    RGB2YUV(r, *bf, bf[width], y1, u1, v1);
    ++bf;

    b = (bf[width - 1] + bf[width + 1]) >> 1;
    // correct:
    // g = (bf[-1] + bf[width] + bf[1]) / 3;
    // faster:
    g = (bf[-1] + bf[1]) >> 1;
    RGB2YUV(*bf, g, b, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // not full data in last columns
  r = (bf[-1] + bf[1]) >> 1;
  RGB2YUV(r, *bf, bf[width], y1, u1, v1);
  ++bf;

  g = (bf[-1] + bf[width]) >> 1;
  RGB2YUV(*bf, g, bf[width - 1], y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  for ( unsigned int h = 1; h < height - 1; h += 2) {

    // b  g  ... line
    // correct: g = (*(bf-width) + bf[1] + bf[width]) / 3;
    // faster:
    g = (*(bf-width) + bf[1]) >> 1;
    r = (*(bf-width+1) + bf[width+1]) >> 1;
    RGB2YUV(r, g, *bf, y1, u1, v1);
    ++bf;

    b = (bf[-1] + bf[1]) >> 1;
    r = (*(bf-width) + bf[width]) >> 1;
    RGB2YUV(r, *bf, g, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);

    for (unsigned int w = 2; w < width - 2; w += 2) {
      g = (*(bf-width) + bf[1] + bf[width] + bf[-1]) >> 2;
      r = (*(bf-width-1) + *(bf-width+1) + bf[width-1] + bf[width+1]) >> 2;
      RGB2YUV(r, g, *bf, y1, u1, v1);
      ++bf;

      b = (bf[-1] + bf[1]) >> 1;
      r = ( *(bf-width) + bf[width]) >> 1;
      RGB2YUV(r, *bf, b, y2, u2, v2);
      ++bf;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    g = (*(bf-width) + bf[1] + bf[width] + bf[-1]) >> 2;
    r = (*(bf-width-1) + *(bf-width+1) + bf[width-1] + bf[width+1]) >> 2;
    RGB2YUV(r, g, *bf, y1, u1, v1);
    ++bf;

    r = (*(bf-width) + bf[width]) >> 1;
    RGB2YUV(r, *bf, bf[-1], y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);


    // g  r  ... line
    b = (bf[width] + *(bf-width)) >> 1;
    RGB2YUV(bf[1], *bf, b, y1, u1, v1);
    ++bf;

    b = (*(bf-width-1) + *(bf-width+1) + bf[width - 1] + bf[width + 1]) >> 2;
    g = (*(bf-width) + bf[1] + bf[width] + bf[-1]) >> 2;
    RGB2YUV(*bf, g, b, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);

    for (unsigned int w = 2; w < width - 2; w += 2) {
      b = (bf[width] + *(bf-width)) >> 1;
      r = (bf[-1] + bf[1]) >> 1;
      RGB2YUV(r, *bf, b, y1, u1, v1);
      ++bf;

      b = (*(bf-width-1) + *(bf-width+1) + bf[width-1] + bf[width+1]) >> 2;
      g = (*(bf-width) + bf[1] + bf[width] + bf[-1]) >> 2;
      RGB2YUV(*bf, g, b, y2, u2, v2);
      ++bf;

      assign(y, u, v, y1, u1, v1, y2, u2, v2);
    }

    b = (bf[width] + *(bf-width)) >> 1;
    r = (bf[-1] + bf[1]) >> 1;
    RGB2YUV(r, *bf, b, y1, u1, v1);
    ++bf;

    b = (*(bf-width-1) + bf[width-1]) >> 1;
    // correct: g = (*(bf-width) + bf[width] + bf[-1]) / 3;
    // faster:
    g = (*(bf-width) + bf[-1]) >> 1;
    RGB2YUV(*bf, g, r, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // last b  g  ... line
  // correct: g = (*(bf-width) + bf[1] + bf[width]) / 3;
  // faster:
  g = (*(bf-width) + bf[1]) >> 1;
  r = *(bf-width+1);
  RGB2YUV(r, g, *bf, y1, u1, v1);
  ++bf;

  b = (bf[-1] + bf[1]) >> 1;
  r = *(bf-width);
  RGB2YUV(r, *bf, b, y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

  for (unsigned int w = 2; w < width - 2; w += 2) {
    // correct: g = (*(bf-width) + bf[1] + bf[-1]) / 3
    // faster:
    g = (*(bf-width) + bf[-1]) >> 1;
    r = (*(bf-width-1) + *(bf-width+1)) >> 1;
    RGB2YUV(r, g, *bf, y1, u1, v1);
    ++bf;

    b = (bf[-1] + bf[1]) >> 1;
    r = *(bf-width);
    RGB2YUV(r, *bf, b, y2, u2, v2);
    ++bf;

    assign(y, u, v, y1, u1, v1, y2, u2, v2);
  }

  // correct: g = (*(bf-width) + bf[1] + bf[-1]) / 3;
  // faster:
  g = (*(bf-width) + bf[-1]) >> 1;
  r = (*(bf-width-1) + *(bf-width+1)) >> 1;
  RGB2YUV(r, g, *bf, y1, u1, v1);
  ++bf;

  r = *(bf-width);
  RGB2YUV(r, *bf, bf[-1], y2, u2, v2);
  ++bf;

  assign(y, u, v, y1, u1, v1, y2, u2, v2);

}


void
bayerGRBG_to_rgb_nearest_neighbour(const unsigned char *bayer, unsigned char *rgb,
                                   unsigned int width, unsigned int height)
{
  for (unsigned int h = 0; h < height; h += 2) {
    // g  r  ... line
    for (unsigned int w = 0; w < width; w += 2) {
      *rgb++ = bayer[1];
      *rgb++ = bayer[width];
      *rgb++ = *bayer;
      ++bayer;

      *rgb++ = *bayer;
      *rgb++ = bayer[-1];
      *rgb++ = bayer[width - 1];
      ++bayer;
    }

    // b  g  ... line
    for (unsigned int w = 0; w < width; w += 2) {
      *rgb++ = *(bayer-width+1);
      *rgb++ = bayer[1];
      *rgb++ = *bayer;
      ++bayer;

      *rgb++ = *(bayer-width);
      *rgb++ = *bayer;
      *rgb++ = bayer[-1];
      ++bayer;
    }
  }
}


void
bayerGRBG_to_rgb_bilinear(const unsigned char *bayer, unsigned char *rgb,
                          unsigned int width, unsigned int height)
{
  // first line is special
  // g  r  ... line
  // not full data in first columns
  *rgb++ = bayer[1];
  *rgb++ = *bayer;
  *rgb++ = bayer[width];
  ++bayer;

  // correct:
  // g = (bayer[-1] + bayer[width] + bayer[1]) / 3;
  // faster:
  *rgb++ = *bayer;
  *rgb++ = (bayer[-1] + bayer[1]) >> 1;
  *rgb++ = (bayer[width - 1] + bayer[width + 1]) >> 1;
  ++bayer;

  // rest of first line
  for (unsigned int w = 2; w < width - 2; w += 2) {
    *rgb++ = (bayer[-1] + bayer[1]) >> 1;
    *rgb++ = *bayer;
    *rgb++ = bayer[width];
    ++bayer;

    // correct:
    // g = (bayer[-1] + bayer[width] + bayer[1]) / 3;
    // faster:
    *rgb++ = *bayer;
    *rgb++ = (bayer[-1] + bayer[1]) >> 1;
    *rgb++ = (bayer[width - 1] + bayer[width + 1]) >> 1;
    ++bayer;
  }

  // not full data in last columns
  *rgb++ = (bayer[-1] + bayer[1]) >> 1;
  *rgb++ = *bayer;
  *rgb++ = bayer[width];
  ++bayer;

  *rgb++ = *bayer;
  *rgb++ = (bayer[-1] + bayer[width]) >> 1;
  *rgb++ = bayer[width - 1];
  ++bayer;

  for ( unsigned int h = 1; h < height - 1; h += 2) {
    // b  g  ... line
    // correct: g = (*(bayer-width) + bayer[1] + bayer[width]) / 3;
    // faster:
    *rgb++ = (*(bayer-width+1) + bayer[width+1]) >> 1;
    *rgb++ = (*(bayer-width) + bayer[1]) >> 1;
    *rgb++ = *bayer;
    ++bayer;

    *rgb++ = (*(bayer-width) + bayer[width]) >> 1;
    *rgb++ = *bayer;
    *rgb++ = (bayer[-1] + bayer[1]) >> 1;
    ++bayer;

    for (unsigned int w = 2; w < width - 2; w += 2) {
      *rgb++ = (*(bayer-width-1) + *(bayer-width+1) + bayer[width-1] + bayer[width+1]) >> 2;
      *rgb++ = (*(bayer-width) + bayer[1] + bayer[width] + bayer[-1]) >> 2;
      *rgb++ = *bayer;
      ++bayer;

      *rgb++ = (*(bayer-width) + bayer[width]) >> 1;
      *rgb++ = *bayer;
      *rgb++ = (bayer[-1] + bayer[1]) >> 1;
      ++bayer;
    }

    *rgb++ = (*(bayer-width-1) + *(bayer-width+1) + bayer[width-1] + bayer[width+1]) >> 2;
    *rgb++ = (*(bayer-width) + bayer[1] + bayer[width] + bayer[-1]) >> 2;
    *rgb++ = *bayer;
    ++bayer;

    *rgb++ = (*(bayer-width) + bayer[width]) >> 1;
    *rgb++ = *bayer;
    *rgb++ = bayer[-1];
    ++bayer;


    // g  r  ... line
    *rgb++ = bayer[1];
    *rgb++ = *bayer;
    *rgb++ = (bayer[width] + *(bayer-width)) >> 1;
    ++bayer;

    *rgb++ = *bayer;
    *rgb++ = (*(bayer-width) + bayer[1] + bayer[width] + bayer[-1]) >> 2;
    *rgb++ = (*(bayer-width-1) + *(bayer-width+1) + bayer[width - 1] + bayer[width + 1]) >> 2;
    ++bayer;

    for (unsigned int w = 2; w < width - 2; w += 2) {
      *rgb++ = (bayer[-1] + bayer[1]) >> 1;
      *rgb++ = *bayer;
      *rgb++ = (bayer[width] + *(bayer-width)) >> 1;
      ++bayer;

      *rgb++ = *bayer;
      *rgb++ = (*(bayer-width) + bayer[1] + bayer[width] + bayer[-1]) >> 2;
      *rgb++ = (*(bayer-width-1) + *(bayer-width+1) + bayer[width-1] + bayer[width+1]) >> 2;
      ++bayer;
    }

    *rgb++ = (bayer[-1] + bayer[1]) >> 1;
    *rgb++ = *bayer;
    *rgb++ = (bayer[width] + *(bayer-width)) >> 1;
    ++bayer;

    // correct: g = (*(bayer-width) + bayer[width] + bayer[-1]) / 3;
    // faster:
    *rgb++ = *bayer;
    *rgb++ = (*(bayer-width) + bayer[-1]) >> 1;
    *rgb++ = (*(bayer-width-1) + bayer[width-1]) >> 1;
    ++bayer;
  }

  // last b  g  ... line
  // correct: g = (*(bayer-width) + bayer[1] + bayer[width]) / 3;
  // faster:
  *rgb++ = *(bayer-width+1);
  *rgb++ = (*(bayer-width) + bayer[1]) >> 1;
  *rgb++ = *bayer;
  ++bayer;

  *rgb++ = *(bayer-width);
  *rgb++ = *bayer;
  *rgb++ = (bayer[-1] + bayer[1]) >> 1;
  ++bayer;

  for (unsigned int w = 2; w < width - 2; w += 2) {
    // correct: g = (*(bayer-width) + bayer[1] + bayer[-1]) / 3
    // faster:
    *rgb++ = (*(bayer-width-1) + *(bayer-width+1)) >> 1;
    *rgb++ = (*(bayer-width) + bayer[-1]) >> 1;
    *rgb++ = *bayer;
    ++bayer;

    *rgb++ = *(bayer-width);
    *rgb++ = *bayer;
    *rgb++ = (bayer[-1] + bayer[1]) >> 1;
    ++bayer;
  }

  // correct: g = (*(bayer-width) + bayer[1] + bayer[-1]) / 3;
  // faster:
  *rgb++ = (*(bayer-width-1) + *(bayer-width+1)) >> 1;
  *rgb++ = (*(bayer-width) + bayer[-1]) >> 1;
  *rgb++ = *bayer;
  ++bayer;

  *rgb++ = *(bayer-width);
  *rgb++ = *bayer;
  *rgb++ = bayer[-1];
  ++bayer;
}


} // end namespace firevision
