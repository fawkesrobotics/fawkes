
/***************************************************************************
 *  yuv.h - YUV specific methods, macros and constants
 *
 *  Created: Sat Aug 12 15:00:12 2006
 *  based on colorspaces.h from Tue Feb 23 13:49:38 2005
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

#include <fvutils/color/yuv.h>
#include <fvutils/color/colorspaces.h>
#include <cstring>

void
iyu1_to_yuy2(unsigned char *src, unsigned char *dest, unsigned int width, unsigned int height)
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
gray8_to_yuy2(unsigned char *src, unsigned char *dest, unsigned int width, unsigned int height)
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
gray8_to_yuv422planar_plainc(unsigned char *src, unsigned char *dst,
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
yuv422planar_copy_uv(unsigned char *src, unsigned char *dst,
		     unsigned int width, unsigned int height,
		     unsigned int x, unsigned int y,
		     unsigned int copy_width, unsigned int copy_height)
{

  register unsigned char *sup = YUV422_PLANAR_U_PLANE(src, width, height) + (x / 2);
  register unsigned char *svp = YUV422_PLANAR_V_PLANE(src, width, height) + (x / 2);

  register unsigned char *dup = YUV422_PLANAR_U_PLANE(dst, width, height) + (x / 2);
  register unsigned char *dvp = YUV422_PLANAR_V_PLANE(dst, width, height) + (x / 2);

  register unsigned int w;
  register unsigned int h;

  unsigned char *lsup = sup, *lsvp = svp, *ldup = dup, *ldvp = dvp;

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
yuv422planar_to_yuv422packed(unsigned char *planar, unsigned char *packed,
			     unsigned int width, unsigned int height)
{
  register unsigned char *y, *u, *v;
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
yuv422packed_to_yuv422planar(unsigned char *packed, unsigned char *planar,
			     unsigned int width, unsigned int height)
{
  register volatile unsigned char *y, *u, *v;
  register int i, iy, iiy;

  unsigned int wh = (width * height);
  unsigned int wh2 = wh / 2;
  y = planar;
  u = planar + wh;
  v = u + wh2;

#pragma omp parallel for private(i, iy, iiy, wh2) shared(y, u, v, packed) schedule(static)
  for (i = 0; i < (int)wh2; ++i) {
    iy  = i << 1;
    iiy = iy << 1;
    u[i]    = packed[iiy];
    y[iy]   = packed[iiy + 1];
    v[i]    = packed[iiy + 2];
    y[iy+1] = packed[iiy + 3];
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
grayscale_yuv422packed(unsigned char *src,   unsigned char *dst,
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
grayscale_yuv422planar(unsigned char *src, unsigned char *dst,
		       unsigned int width, unsigned int height)
{
  memcpy(dst, src, width * height);
  memset(dst + width * height, 128, width * height);
}

