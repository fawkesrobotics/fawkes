
/***************************************************************************
 *  colorspaces.h - This header defines utility functions to deal with
 *                  color spaces
 *
 *  Generated: Tue Feb 23 13:49:38 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_UTILS_COLOR_COLORSPACES_H_
#define __FIREVISION_UTILS_COLOR_COLORSPACES_H_

#include <sys/types.h>

/** Color spaces. */
typedef enum {
  CS_UNKNOWN         =  0,	/**< Unknown color space */
  RGB                =  1,	/**< RGB, three bytes per pixel, one byte per color, ordered
				 * line by line */
  YUV411_PACKED      =  2,	/**< YUV image with 4:1:1 sampling, byte order U Y0 Y1 V Y2 Y3 */
  YUV411_PLANAR      =  3,	/**< YUV image with 4:1:1 sampling, first Y plane, then U then V plane */
  YUY2               =  4,	/**< YUV image with 4:2:2 sampling, byte order Y0 U Y1 V */
  BGR                =  5,	/**< RGB, 3 bytes per pixel, one byte per color, ordererd
				 * line by line, pixels orderd B G R */
  YUV422_PACKED      =  6,	/**< YUV image with 4:2:2 sampling, byte order U Y0 V Y1 */
  YUV422_PLANAR      =  7,	/**< YUV image with 4:2:2 sampling, first Y plane, then U then V plane */
  GRAY8              =  8,	/**< plain gray buffer, one byte per pixel */
  RGB_WITH_ALPHA     =  9,	/**< RGB with alpha, 4 bytes per pixel, byte order R G B A */
  BGR_WITH_ALPHA     = 10,	/**< RGB with alpha, 4 bytes per pixel, byte order B G R A */
  BAYER_MOSAIC_RGGB  = 11,	/**< Image has RGGB bayer pattern */
  BAYER_MOSAIC_GBRG  = 12,	/**< Image has GBRG bayer pattern */
  BAYER_MOSAIC_GRBG  = 13,	/**< Image has GRBG bayer pattern */
  BAYER_MOSAIC_BGGR  = 14,	/**< Image has BGGR bayer pattern */
  RAW16              = 15,	/**< Raw image, 2 bytes per pixel, format depends on camera */
  RAW8               = 16,	/**< Raw image, 1 byte per pixel, format depends on camera */
  MONO8              = 17,	/**< Like GRAY8 */
  MONO16             = 18,	/**< Gray-scale image, 2 bytes per pixel */
  YUV444_PACKED      = 19,	/**< Full sampled YUV, byte order Y U V */
  YVU444_PACKED      = 20,	/**< Full sampled YUV, byte order Y V U */
  COLORSPACE_N       = 21	/**< number of colorspaces */
} colorspace_t;


inline size_t
colorspace_buffer_size(colorspace_t cspace, unsigned int width, unsigned int height)
{
  switch (cspace) {
  case RGB:
  case BGR:
  case YUV444_PACKED:
  case YVU444_PACKED:
    return (width * height * 3);
 
  case RGB_WITH_ALPHA:
  case BGR_WITH_ALPHA:
    return (width * height * 4);

  case YUV411_PACKED:
  case YUV411_PLANAR:
    return (width * height * 3 / 2);
    
  case YUY2:
    return (width * height * 2);
    
  case YUV422_PACKED:
  case YUV422_PLANAR:
    return (width * height * 2);

  case RAW16:
    return (width * height * 2);
    
  case GRAY8:
  case BAYER_MOSAIC_RGGB:
  case BAYER_MOSAIC_GBRG:
  case BAYER_MOSAIC_GRBG:
  case BAYER_MOSAIC_BGGR:
    return (width * height);
    
  default:
    return 0;
  }
}


colorspace_t     colorspace_by_name(const char *colorspace);
const char *     colorspace_to_string(colorspace_t colorspace);
unsigned char *  malloc_buffer(colorspace_t colorspace, unsigned int width, unsigned int height);

#endif
