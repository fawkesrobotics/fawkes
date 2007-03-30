
/***************************************************************************
 *  bayer.cpp - methods to convert bayer mosaic images to other formats
 *
 *  Generated: Fri Aug 11 00:03:32 2006
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

#include <fvutils/color/bayer.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/rgbyuv.h>

/* The basic information has been taken from
 * http://www-ise.stanford.edu/~tingchen/
 */

#define assign(y, u, v, y1, u1, v1, y2, u2, v2) { \
    *y++ = y1;					  \
    *y++ = y2;					  \
    *u++ = ((u1 + u2) >> 1);			  \
    *v++ = ((v1 + v2) >> 1); }  


void
bayerGBRG_to_yuv422planar_nearest_neighbour(unsigned char *bayer, unsigned char *yuv,
					    unsigned int width, unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  unsigned char *b = bayer;

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
bayerGBRG_to_yuv422planar_bilinear(unsigned char *bayer, unsigned char *yuv,
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
    RGB2YUV(r, *bf, g, y2, u2, v2);
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
bayerGBRG_to_yuv422planar_bilinear2(unsigned char *bayer, unsigned char *yuv,
				   unsigned int width, unsigned int height)
{
  unsigned char *y = yuv;
  unsigned char *u = YUV422_PLANAR_U_PLANE(yuv, width, height);
  unsigned char *v = YUV422_PLANAR_V_PLANE(yuv, width, height);
  unsigned char *bf = bayer;

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
bayerGBRG_to_yuv422planar_bilinear2(unsigned char *bayer, unsigned char *yuv,
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
