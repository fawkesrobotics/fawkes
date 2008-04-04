
/***************************************************************************
 *  yuv.h - YUV specific methods, macros and constants
 *
 *  Created: Sat Aug 12 14:36:28 2006
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
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FIREVISION_UTILS_COLOR_YUV_H
#define __FIREVISION_UTILS_COLOR_YUV_H


#define YUV422PA_MACROPIXEL_AT(YUV, width, x, y) ((unsigned char*)YUV + (y)*(width)*2 + ((x)-((x)%2))*2)

#define YUV422_PLANAR_Y_AT(YUV, width, x, y)	\
  *(YUV + (y) * (width) + (x));

#define YUV422_PLANAR_U_AT(YUV, width, height, x, y)		\
  *(YUV + ((width) * (height)) + (((y) * (width) + (x))/ 2));

#define YUV422_PLANAR_V_AT(YUV, width, height, x, y)			\
  *(YUV + ((width) * (height)) + (((width) * (height) + (y) * (width) + (x)) / 2)); \
  
#define YUV422_PLANAR_YUV(YUV, width, height, x, y, yp, up, vp)		\
  {									\
    yp = YUV422_PLANAR_Y_AT(YUV, width, x, y);				\
    up = YUV422_PLANAR_U_AT(YUV, width, height, x, y);			\
    vp = YUV422_PLANAR_V_AT(YUV, width, height, x, y);			\
  }

#define YUV422_PLANAR_U_PLANE(YUV, width, height) (YUV + (width) * (height))
#define YUV422_PLANAR_V_PLANE(YUV, width, height) (YUV + ((width) * (height)) + ((width) * (height) / 2))

/** YUV pixel. */
typedef struct {
  unsigned char Y;	/**< Y component */
  unsigned char U;	/**< U component */
  unsigned char V;	/**< V component */
} YUV_t;


/** Convert IYU1 to IYU2
 * @param src src buffer
 * @param dest destination buffer
 * @param width image width
 * @param height image height
 */
void iyu1_to_yuy2(unsigned char *src, unsigned char *dest,
		  unsigned int width, unsigned int height);


/** 8-Bit gray to YUY2 conversion
 * This function takes the gray value as Y and sets U and V to 128.
 */
void gray8_to_yuy2(unsigned char *src, unsigned char *dest,
		   unsigned int width, unsigned int height);


/** 8-Bit gray to YUV422_PLANAR
 */
void gray8_to_yuv422planar_plainc(unsigned char *src, unsigned char *dst,
				  unsigned int width, unsigned int height);


/** Copy part of the U anv V planes of a YUV422planar image to another
 */
void yuv422planar_copy_uv(unsigned char *src, unsigned char *dst,
			  unsigned int width, unsigned int height,
			  unsigned int x, unsigned int y,
			  unsigned int copy_width, unsigned int copy_height);



/** Convert YUV422_PLANAR images to YUV422_PACKED
 */
void yuv422planar_to_yuv422packed(unsigned char *planar, unsigned char *packed,
				  unsigned int width, unsigned int height);


/** Convert YUV422_PACKED images to YUV422_PLANAR
 */
void yuv422packed_to_yuv422planar(unsigned char *packed, unsigned char *planar,
				  unsigned int width, unsigned int height);



void yuv422planar_erase_y_plane(unsigned char *yuv, unsigned int width, unsigned int height);


void yuv422planar_erase_u_plane(unsigned char *yuv, unsigned int width, unsigned int height);


void yuv422planar_erase_v_plane(unsigned char *yuv, unsigned int width, unsigned int height);


void grayscale_yuv422packed(unsigned char *src,   unsigned char *dst,
			    unsigned int   width, unsigned int   height);


void grayscale_yuv422planar(unsigned char *src, unsigned char *dst,
			    unsigned int width, unsigned int height);


inline void
convert_line_yuv422planar_to_yuv444packed(unsigned char *src, unsigned char *dst,
					  unsigned int width, unsigned int height,
					  unsigned int src_line, unsigned int dst_line)
{
  register unsigned int i = 0;
  register YUV_t *y1, *y2;
  register unsigned char *yp, *up, *vp;

  yp = src + (width * src_line);
  up = YUV422_PLANAR_U_PLANE(src, width, height) + (width * src_line / 2);
  vp = YUV422_PLANAR_V_PLANE(src, width, height) + (width * src_line / 2);

  dst += 3 * width * dst_line;

  while (i < width) {
    y1 = (YUV_t *)dst;
    dst += 3;
    y2 = (YUV_t *)dst;
    dst += 3;

    y1->Y = *yp++;
    y1->U = *up;
    y1->V = *vp;

    y2->Y = *yp++;
    y2->U = *up++;
    y2->V = *vp++;

    i += 2;
  }
}

#endif
