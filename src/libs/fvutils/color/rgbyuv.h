
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

#ifndef __FIREVISION_UTILS_COLOR_RGBYUV_H
#define __FIREVISION_UTILS_COLOR_RGBYUV_H

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


#define RGB2YUV(r, g, b, y, u, v) { 				\
    y = (306*r + 601*g + 117*b)  >> 10;				\
    u = ((-172*r - 340*g + 512*b) >> 10)  + 128;		\
    v = ((512*r - 429*g - 83*b) >> 10) + 128;			\
    y = y < 0 ? 0 : y;						\
    u = u < 0 ? 0 : u;						\
    v = v < 0 ? 0 : v;						\
    y = y > 255 ? 255 : y;					\
    u = u > 255 ? 255 : u;					\
    v = v > 255 ? 255 : v; }

/* Alternative from libdc1394
  y = (306*r + 601*g + 117*b)  >> 10;			\
  u = ((-172*r - 340*g + 512*b) >> 10)  + 128;\
  v = ((512*r - 429*g - 83*b) >> 10) + 128;\

  Original:
  y = ((9798*(r) + 19235*(g) + 3736*(b))  >> 15);		\
  u = ((-4784*(r) - 9437*(g) + 14221*(b)) >> 15) + 128;		\
  v = ((20218*(r) - 16941*(g) - 3277*(b)) >> 15) + 128;		\

*/


void rgb_to_yuy2(const unsigned char *RGB, unsigned char *YUV,
		 unsigned int width, unsigned int height);


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
void rgb_to_yuv411packed_plainc(const unsigned char *RGB, unsigned char *YUV,
				unsigned int width, unsigned int height);

/* Convert a line of a RGB buffer to a line in a planar YUV422 buffer, see above for general
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
void convert_line_rgb_to_yuv422planar(const unsigned char *RGB, unsigned char *YUV,
				      unsigned int width, unsigned int height,
				      unsigned int rgb_line, unsigned int yuv_line);


/* Convert an RGB buffer to a planar YUV422 buffer, see above for general notes about color space
 * conversion from RGB to YUV
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 */
void rgb_to_yuv422planar_plainc(const unsigned char *RGB, unsigned char *YUV,
				unsigned int width, unsigned int height);

/* Convert a planar RGB buffer to a packed YUV422 buffer.
 * See above for general notes about color space
 * conversion from RGB to YUV
 * @param RGB unsigned char array that contains the color planes
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 */
void rgb_planar_to_yuv422packed_plainc(const unsigned char *rgb_planar, unsigned char *YUV,
				       unsigned int width, unsigned int height);

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
void convert_line_rgb_to_yuv422packed(const unsigned char *RGB, unsigned char *YUV,
				      unsigned int width, unsigned int height,
				      unsigned int rgb_line, unsigned int yuv_line);

/* Convert an RGB buffer to a packed YUV422 buffer, see above for general notes about color space
 * conversion from RGB to YUV
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 */
void rgb_to_yuv422packed_plainc(const unsigned char *RGB, unsigned char *YUV,
				unsigned int width, unsigned int height);

/* Convert an BGR buffer to a planar YUV422 buffer, see above for general notes about color space
 * conversion from RGB to YUV
 * @param RGB unsigned char array that contains the pixels, pixel after pixel, 3 bytes per pixel
 *            (thus this is a 24bit RGB with one byte per color) line by line.
 * @param YUV where the YUV output will be written to, will have 4 pixels in 6 byte macro pixel, line after
 *            line
 * @param width Width of the image contained in the RGB buffer
 * @param height Height of the image contained in the RGB buffer
 */
void bgr_to_yuv422planar_plainc(const unsigned char *BGR, unsigned char *YUV,
				unsigned int width, unsigned int height);


} // end namespace firevision

#endif
