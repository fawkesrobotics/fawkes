
/***************************************************************************
 *  yuv.h - YUV specific methods, macros and constants
 *
 *  Created: Sat Aug 12 14:36:28 2006
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

#ifndef __FIREVISION_UTILS_COLOR_YUV_H
#define __FIREVISION_UTILS_COLOR_YUV_H

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


#define YUV422PA_MACROPIXEL_AT(YUV, width, x, y) ((unsigned char*)YUV + (y)*(width)*2 + ((x)-((x)%2))*2)

#define YUV422_PLANAR_Y_AT(YUV, width, x, y)	\
  *(YUV + (y) * (width) + (x))

#define YUV422_PLANAR_U_AT(YUV, width, height, x, y)		\
  *(YUV + ((width) * (height)) + (((y) * (width) + (x))/ 2))

#define YUV422_PLANAR_V_AT(YUV, width, height, x, y)			\
  *(YUV + ((width) * (height)) + (((width) * (height) + (y) * (width) + (x)) / 2))

#define YUV422_PLANAR_YUV(YUV, width, height, x, y, yp, up, vp)		\
  {									\
    yp = YUV422_PLANAR_Y_AT(YUV, width, x, y);				\
    up = YUV422_PLANAR_U_AT(YUV, width, height, x, y);			\
    vp = YUV422_PLANAR_V_AT(YUV, width, height, x, y);			\
  }

#define YUV422_PLANAR_U_PLANE(YUV, width, height) (YUV + (width) * (height))
#define YUV422_PLANAR_V_PLANE(YUV, width, height) (YUV + ((width) * (height)) + ((width) * (height) / 2))

/** YUV pixel. */
typedef struct YUV_t_struct{
  unsigned char Y;	/**< Y component */
  unsigned char U;	/**< U component */
  unsigned char V;	/**< V component */

  /** Standard constructor
   * @param y Y component
   * @param u U component
   * @param v V component
   */
  YUV_t_struct(unsigned char y = 127, unsigned char u = 127, unsigned char v = 127)
  {
    Y = y;
    U = u;
    V = v;
  }

  static YUV_t_struct white()   { return YUV_t_struct(255, 127, 127); } /**< @return white color */
  static YUV_t_struct black()   { return YUV_t_struct(  0, 127, 127); } /**< @return black color */
  static YUV_t_struct green()   { return YUV_t_struct( 64,  95,  85); } /**< @return green color */
  static YUV_t_struct cyan()    { return YUV_t_struct(178, 170,   0); } /**< @return cyan color */
  static YUV_t_struct magenta() { return YUV_t_struct(105, 212, 234); } /**< @return magenta color */
  static YUV_t_struct gray()    { return YUV_t_struct(127, 127, 127); } /**< @return gray color */
  static YUV_t_struct orange()  { return YUV_t_struct(150,  43, 202); } /**< @return orange color */
  static YUV_t_struct yellow()  { return YUV_t_struct(245,   0, 148); } /**< @return yellow color */
  static YUV_t_struct blue()    { return YUV_t_struct( 29, 255, 107); } /**< @return blue color */
  static YUV_t_struct red()     { return YUV_t_struct( 75,  85, 255); } /**< @return red color */
} YUV_t;


/** Convert IYU1 to IYU2
 * @param src src buffer
 * @param dest destination buffer
 * @param width image width
 * @param height image height
 */
void iyu1_to_yuy2(const unsigned char *src, unsigned char *dest,
		  unsigned int width, unsigned int height);


/** 8-Bit gray to YUY2 conversion
 * This function takes the gray value as Y and sets U and V to 128.
 */
void gray8_to_yuy2(const unsigned char *src, unsigned char *dest,
		   unsigned int width, unsigned int height);


/** 8-Bit gray to YUV422_PLANAR
 */
void gray8_to_yuv422planar_plainc(const unsigned char *src, unsigned char *dst,
				  unsigned int width, unsigned int height);


/** Copy part of the U anv V planes of a YUV422planar image to another
 */
void yuv422planar_copy_uv(const unsigned char *src, unsigned char *dst,
			  unsigned int width, unsigned int height,
			  unsigned int x, unsigned int y,
			  unsigned int copy_width, unsigned int copy_height);



/** Convert YUV422_PLANAR images to YUV422_PACKED
 */
void yuv422planar_to_yuv422packed(const unsigned char *planar, unsigned char *packed,
				  unsigned int width, unsigned int height);

/** Convert YUV422_PLANAR_QUARTER images to YUV422_PACKED
 */
void yuv422planar_quarter_to_yuv422packed(const unsigned char *planar,
					  unsigned char *packed,
					  const unsigned int width,
					  const unsigned int height);

/** Convert YUV422_PLANAR_QUARTER images to YUV422_PLANAR  */
void yuv422planar_quarter_to_yuv422planar(const unsigned char *planar,
					  unsigned char *packed,
					  const unsigned int width,
					  const unsigned int height);


/** Convert YUV422_PACKED images to YUV422_PLANAR
 */
void yuv422packed_to_yuv422planar(const unsigned char *packed, unsigned char *planar,
				  unsigned int width, unsigned int height);

/** Convert YUY2 images to YUV422_PLANAR
 */
void yuy2_to_yuv422planar(const unsigned char *packed, unsigned char *planar,
				  unsigned int width, unsigned int height);

/** Convert YUY2 images to quarter-sized YUV422_PLANAR buffer.
 */
void yuy2_to_yuv422planar_quarter(const unsigned char *packed, unsigned char *planar,
				  const unsigned int width, const unsigned int height);

/** Convert YVY2 images to YUV422_PLANAR
 */
void yvy2_to_yuv422planar(const unsigned char *packed, unsigned char *planar,
				  unsigned int width, unsigned int height);

/** Convert YUV444_PACKED images to YUV422_PLANAR
 */
void yuv444packed_to_yuv422planar(const unsigned char *yuv444, unsigned char *yuv422,
				  unsigned int width, unsigned int height);

void yuv444packed_to_yuv422packed(const unsigned char *yuv444, unsigned char *yuv422,
				  unsigned int width, unsigned int height);

void yvu444packed_to_yuv422planar(const unsigned char *yuv444, unsigned char *yuv422,
				  unsigned int width, unsigned int height);

void yvu444packed_to_yuv422packed(const unsigned char *yuv444, unsigned char *yuv422,
				  unsigned int width, unsigned int height);



void yuv422planar_erase_y_plane(unsigned char *yuv, unsigned int width, unsigned int height);


void yuv422planar_erase_u_plane(unsigned char *yuv, unsigned int width, unsigned int height);


void yuv422planar_erase_v_plane(unsigned char *yuv, unsigned int width, unsigned int height);


void grayscale_yuv422packed(const unsigned char *src,   unsigned char *dst,
			    unsigned int   width, unsigned int   height);


void grayscale_yuv422planar(const unsigned char *src, unsigned char *dst,
			    unsigned int width, unsigned int height);


inline void
convert_line_yuv422planar_to_yuv444packed(const unsigned char *src, unsigned char *dst,
					  unsigned int width, unsigned int height,
					  unsigned int src_line, unsigned int dst_line)
{
  register unsigned int i = 0;
  register YUV_t *y1, *y2;
  register const unsigned char *yp, *up, *vp;

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

} // end namespace firevision

#endif
