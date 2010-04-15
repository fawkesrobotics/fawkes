
/***************************************************************************
 *  yuv.h - YUV specific methods, macros and constants
 *
 *  Created: Sat Aug 12 15:00:12 2006
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

#include <fvutils/color/yuv.h>
#include <fvutils/color/colorspaces.h>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void
iyu1_to_yuy2(const unsigned char *src, unsigned char *dest, unsigned int width, unsigned int height)
{
  unsigned int i=0, j=0;
  register int y0, y1, y2, y3, u, v;
  while (i < width * height * 3 / 2) {
    u = src[i++];
    y0 = src[i++];
    y1 = src[i++];
    v = src[i++];
    y2 = src[i++];
    y3 = src[i++];

    dest[j++] = y0;
    dest[j++] = u;
    dest[j++] = y1;
    dest[j++] = v;

    dest[j++] = y2;
    dest[j++] = u;
    dest[j++] = y3;
    dest[j++] = v;
  }
}


/** 8-Bit gray to YUY2 conversion
 * This function takes the gray value as Y and sets U and V to 128.
 */
void
gray8_to_yuy2(const unsigned char *src, unsigned char *dest, unsigned int width, unsigned int height)
{
  register unsigned int i=0, j=0;
  while (i < width * height) {
    dest[j++] = src[i++];
    dest[j++] = 128;
    dest[j++] = src[i++];
    dest[j++] = 128;
  }
}


/** 8-Bit gray to YUV422_PLANAR
 */
void
gray8_to_yuv422planar_plainc(const unsigned char *src, unsigned char *dst,
			     unsigned int width, unsigned int height)
{
  // copy Y plane
  memcpy(dst, src, width * height);
  // set U and V plane
  memset(YUV422_PLANAR_U_PLANE(dst, width, height), 128, width * height);
}



/** Copy part of the U anv V planes of a YUV422planar image to another
 */
void
yuv422planar_copy_uv(const unsigned char *src, unsigned char *dst,
		     unsigned int width, unsigned int height,
		     unsigned int x, unsigned int y,
		     unsigned int copy_width, unsigned int copy_height)
{

  register const unsigned char *sup = YUV422_PLANAR_U_PLANE(src, width, height) + (x / 2);
  register const unsigned char *svp = YUV422_PLANAR_V_PLANE(src, width, height) + (x / 2);

  register unsigned char *dup = YUV422_PLANAR_U_PLANE(dst, width, height) + (x / 2);
  register unsigned char *dvp = YUV422_PLANAR_V_PLANE(dst, width, height) + (x / 2);

  register unsigned int w;
  register unsigned int h;

  unsigned const char *lsup = sup, *lsvp = svp, *ldup = dup, *ldvp = dvp;

  for (h = 0; h < copy_height; ++h) {
    for ( w = 0; w < copy_width; w += 2 ) {
      *dup++ = *sup++;
      *dvp++ = *svp++;
    }
    lsup += width / 2;
    lsvp += width / 2;
    ldup += width / 2;
    ldvp += width / 2;
  }
}


void
yuv422planar_to_yuv422packed(const unsigned char *planar, unsigned char *packed,
			     unsigned int width, unsigned int height)
{
  register const unsigned char *y, *u, *v;
  register unsigned int i;

  y = planar;
  u = planar + (width * height);
  v = u + (width * height / 2);

  for (i = 0; i < (width * height / 2); ++i) {
    *packed++ = *u++;
    *packed++ = *y++;
    *packed++ = *v++;
    *packed++ = *y++;
  }
}

void
yuv422planar_quarter_to_yuv422packed(const unsigned char *planar, unsigned char *packed,
				     const unsigned int width,
				     const unsigned int height)
{
  volatile const unsigned char *y, *u, *v;
  register unsigned int w, h;

  const unsigned int w_h_4 = (width * height) / 4;
  const unsigned int w_h_8 = (width * height) / 8;
  const unsigned int w_t_2   = width * 2;
  const unsigned int w_b_2   = width / 2;
  const unsigned int w_b_4   = width / 4;


  for (h = 0; h < height / 2; ++h) {
    y = planar + (h * w_b_2);
    u = planar + w_h_4 + (h * w_b_4);
    v = planar + w_h_4 + w_h_8 + (h * w_b_4);

    for (w = 0; w < w_b_4; ++w) {
      packed[h * w_t_2 + w * 4    ] = *u++;
      packed[h * w_t_2 + w * 4 + 1] = *y++;
      packed[h * w_t_2 + w * 4 + 2] = *v++;
      packed[h * w_t_2 + w * 4 + 3] = *y++;
    }
  }
}


/* Convert quarter YUV422 planar buffer to plain YUV422 planar.
 * @param quarter input buffer in YUV422_PLANAR_QUARTER
 * @param output buffer in YUV422_PLANAR
 * @param width width of the image (width of YUV422_PLANAR image)
 * @param height height of the image (height of YUV422_PLANAR image)
 */
void
yuv422planar_quarter_to_yuv422planar(const unsigned char *quarter,
				     unsigned char *planar,
				     const unsigned int width,
				     const unsigned int height)
{
  volatile const unsigned char *y, *u, *v;
  register unsigned int w, h;

  const unsigned int w_h_4 = (width * height) / 4;
  const unsigned int w_h_8 = (width * height) / 8;
  //const unsigned int w_t_2   = width * 2;
  const unsigned int w_b_2   = width / 2;
  const unsigned int w_b_4   = width / 4;

  unsigned char *yp, *up, *vp, t;
  yp = planar;
  up = YUV422_PLANAR_U_PLANE(planar, width, height);
  vp = YUV422_PLANAR_V_PLANE(planar, width, height);

  for (h = 0; h < height / 2; ++h) {
    y  = quarter + (h * w_b_2);
    u  = quarter + w_h_4 + (h * w_b_4);
    v  = quarter + w_h_4 + w_h_8 + (h * w_b_4);

    for (w = 0; w < w_b_4; ++w) {
      t = *y++;
      *yp++ = t;
      *yp++ = t;
      t = *y++;
      *yp++ = t;
      *yp++ = t;
      t = *u++;
      *up++ = t;
      *up++ = t;
      t = *v++;
      *vp++ = t;
      *vp++ = t;
    }

    memcpy(yp, yp - width, width);
    memcpy(up, up - w_b_2, w_b_2);
    memcpy(vp, vp - w_b_2, w_b_2);
    yp += width;
    up += w_b_2;
    vp += w_b_2;
  }

}

void
yuv422packed_to_yuv422planar(const unsigned char *packed, unsigned char *planar,
			     unsigned int width, unsigned int height)
{
  register volatile unsigned char *y, *u, *v;
  register int i, iy, iiy;

  unsigned int wh = (width * height);
  int wh2 = wh >> 1;
  y = planar;
  u = planar + wh;
  v = u + wh2;

#ifdef _OPENMP
  #pragma omp parallel for firstprivate(wh2) private(i, iy, iiy) shared(y, u, v, packed) schedule(static)
#endif
  for (i = 0; i < wh2; ++i) {
    iy  = i << 1;
    iiy = iy << 1;
    u[i]    = packed[iiy];
    y[iy]   = packed[iiy + 1];
    v[i]    = packed[iiy + 2];
    y[iy+1] = packed[iiy + 3];
  }
}


void
yuy2_to_yuv422planar(const unsigned char *packed, unsigned char *planar,
			     unsigned int width, unsigned int height)
{
  register volatile unsigned char *y, *u, *v;
  register int i, iy, iiy;

  unsigned int wh = (width * height);
  int wh2 = wh >> 1;
  y = planar;
  u = planar + wh;
  v = u + wh2;

#ifdef _OPENMP
  #pragma omp parallel for firstprivate(wh2) private(i, iy, iiy) shared(y, u, v, packed) schedule(static)
#endif
  for (i = 0; i < wh2; ++i) {
    iy  = i << 1;
    iiy = iy << 1;
    y[iy]   = packed[iiy];
    u[i]    = packed[iiy + 1];
    y[iy+1] = packed[iiy + 2];
    v[i]    = packed[iiy + 3];
  }
}


void
yvy2_to_yuv422planar(const unsigned char *packed, unsigned char *planar,
			     unsigned int width, unsigned int height)
{
  register volatile unsigned char *y, *u, *v;
  register int i, iy, iiy;

  unsigned int wh = (width * height);
  int wh2 = wh >> 1;
  y = planar;
  u = planar + wh;
  v = u + wh2;

#ifdef _OPENMP
  #pragma omp parallel for firstprivate(wh2) private(i, iy, iiy) shared(y, u, v, packed) schedule(static)
#endif
  for (i = 0; i < wh2; ++i) {
    iy  = i << 1;
    iiy = iy << 1;
    y[iy]   = packed[iiy];
    v[i]    = packed[iiy + 1];
    y[iy+1] = packed[iiy + 2];
    u[i]    = packed[iiy + 3];
  }
}


void
yuy2_to_yuv422planar_quarter(const unsigned char *packed, unsigned char *planar,
			     const unsigned int width, const unsigned int height)
{
  register volatile unsigned char *y, *u, *v;
  register unsigned int h, w;

  unsigned int wh = (width * height);
  y = planar;
  u = planar + (wh / 4);
  v = u + (wh / 8);

  const unsigned int w_b_2 = width / 2;
  const unsigned int w_b_4 = width / 4;
  const unsigned int w_t_2 = width * 2;
  unsigned int packpix;

  for (h = 0; h < height / 2; ++h) {
    for (w = 0; w < width; w += 4) {
      packpix = (h * w_t_2 + w) * 2;
      y[h * w_b_2 + w / 2    ] = (packed[packpix + 0] + packed[packpix + 2]) / 2;
      u[h * w_b_4 + w / 4    ] = (packed[packpix + 1] + packed[packpix + 5]) / 2;
      y[h * w_b_2 + w / 2 + 1] = (packed[packpix + 4] + packed[packpix + 6]) / 2;
      v[h * w_b_4 + w / 4    ] = (packed[packpix + 3] + packed[packpix + 7]) / 2;
    }
  }
}


void
yuv444packed_to_yuv422planar(const unsigned char *yuv444, unsigned char *yuv422,
			     unsigned int width, unsigned int height)
{
  register volatile unsigned char *y, *u, *v;
  register int i, iy, iiy;

  unsigned int wh = (width * height);
  int wh2 = wh >> 1;
  y = yuv422;
  u = yuv422 + wh;
  v = u + wh2;

#ifdef ___OPENMP
  #pragma omp parallel for firstprivate(wh2) private(i, iy, iiy) shared(y, u, v, yuv444) schedule(static)
#endif
  for (i = 0; i < wh2; ++i) {
    iy  = i << 1;
    iiy = i * 6;
    y[iy]   = yuv444[iiy];
    y[iy+1] = yuv444[iiy + 3];
    u[i]    = (yuv444[iiy + 1] + yuv444[iiy + 4]) >> 1;
    v[i]    = (yuv444[iiy + 2] + yuv444[iiy + 5]) >> 1;
  }
}


void
yuv444packed_to_yuv422packed(const unsigned char *yvu444, unsigned char *yuv422,
			     unsigned int width, unsigned int height)
{
  register int i, iiy;

  unsigned int wh = (width * height);
  int wh2 = wh >> 1;

#ifdef ___OPENMP
#  pragma omp parallel for firstprivate(wh2) private(i, iiy) shared(yuv422, yvu444) schedule(static)
#endif
  for (i = 0; i < wh2; i += 4) {
    iiy = i * 6;
    yuv422[i] = (yvu444[iiy + 1] + yvu444[iiy + 4]) >> 1;
    yuv422[i+1] = yvu444[iiy];
    yuv422[i+2]   = (yvu444[iiy + 2] + yvu444[iiy + 5]) >> 1;
    yuv422[i+3] = yvu444[iiy + 3];
  }
}


void
yvu444packed_to_yuv422planar(const unsigned char *yvu444, unsigned char *yuv422,
			     unsigned int width, unsigned int height)
{
  register volatile unsigned char *y, *u, *v;
  register int i, iy, iiy;

  unsigned int wh = (width * height);
  int wh2 = wh >> 1;
  y = yuv422;
  u = yuv422 + wh;
  v = u + wh2;

#ifdef ___OPENMP
#  pragma omp parallel for firstprivate(wh2) private(i, iy, iiy) shared(y, u, v, yvu444) schedule(static)
#endif
  for (i = 0; i < wh2; ++i) {
    iy  = i << 1;
    iiy = i * 6;
    y[iy]   = yvu444[iiy];
    y[iy+1] = yvu444[iiy + 3];
    u[i]    = (yvu444[iiy + 2] + yvu444[iiy + 5]) >> 1;
    v[i]    = (yvu444[iiy + 1] + yvu444[iiy + 4]) >> 1;
  }
}


void
yvu444packed_to_yuv422packed(const unsigned char *yvu444, unsigned char *yuv422,
			     unsigned int width, unsigned int height)
{
  register int i, iiy;

  unsigned int wh = (width * height);
  int wh2 = wh >> 1;

#ifdef ___OPENMP
#  pragma omp parallel for firstprivate(wh2) private(i, iiy) shared(yuv422, yvu444) schedule(static)
#endif
  for (i = 0; i < wh2; i += 4) {
    iiy = i * 6;
    yuv422[i]   = (yvu444[iiy + 2] + yvu444[iiy + 5]) >> 1;
    yuv422[i+1] = yvu444[iiy];
    yuv422[i+2] = (yvu444[iiy + 1] + yvu444[iiy + 4]) >> 1;
    yuv422[i+3] = yvu444[iiy + 3];
  }
}


void
yuv422planar_erase_y_plane(unsigned char *yuv, unsigned int width, unsigned int height)
{
  memset(yuv, 128, (width * height));
}


void
yuv422planar_erase_u_plane(unsigned char *yuv, unsigned int width, unsigned int height)
{
  memset(yuv + (width * height), 128, (width * height / 2));
}


void
yuv422planar_erase_v_plane(unsigned char *yuv, unsigned int width, unsigned int height)
{
  memset(yuv + (width * height * 3/2), 128, (width * height / 2));
}


void
grayscale_yuv422packed(const unsigned char *src,   unsigned char *dst,
		       unsigned int   width, unsigned int   height)
{
  unsigned int p = 0;
  unsigned int d = 0;
  while (p < colorspace_buffer_size(YUV422_PACKED, width, height)) {
    if ( (p % 2) == 0 ) {
      //dst[p] = 128;
    } else {
      dst[d++] = src[p];
    }
    p += 1;
  }
}


void
grayscale_yuv422planar(const unsigned char *src, unsigned char *dst,
		       unsigned int width, unsigned int height)
{
  memcpy(dst, src, width * height);
  memset(dst + width * height, 128, width * height);
}

} // end namespace firevision
