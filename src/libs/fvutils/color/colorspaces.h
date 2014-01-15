
/***************************************************************************
 *  colorspaces.h - This header defines utility functions to deal with
 *                  color spaces
 *
 *  Generated: Tue Feb 23 13:49:38 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Color spaces.
 * Color spaces have their name for historical reasons, but the proper
 * name would be buffer format. A colorspace defines a particular layout
 * of a memory buffer containing an image (or point cloud).
 */
typedef enum {
  CS_UNKNOWN            =  0,	/**< Unknown color space */
  RGB                   =  1,	/**< RGB, three bytes per pixel, one byte per color, ordered
				 * line by line */
  YUV411_PACKED         =  2,	/**< YUV image with 4:1:1 sampling, byte order U Y0 Y1 V Y2 Y3 */
  YUV411_PLANAR         =  3,	/**< YUV image with 4:1:1 sampling, first Y plane, then U then V plane */
  YUY2                  =  4,	/**< YUV image with 4:2:2 sampling, byte order Y0 U Y1 V */
  BGR                   =  5,	/**< RGB, 3 bytes per pixel, one byte per color, ordererd
				 * line by line, pixels orderd B G R */
  YUV422_PACKED         =  6,	/**< YUV image with 4:2:2 sampling, byte order U Y0 V Y1 */
  YUV422_PLANAR         =  7,	/**< YUV image with 4:2:2 sampling, first Y plane, then U then V plane */
  GRAY8                 =  8,	/**< plain gray buffer, one byte per pixel */
  RGB_WITH_ALPHA        =  9,	/**< RGB with alpha, 4 bytes per pixel, byte order R G B A */
  BGR_WITH_ALPHA        = 10,	/**< RGB with alpha, 4 bytes per pixel, byte order B G R A */
  BAYER_MOSAIC_RGGB     = 11,	/**< Image has RGGB bayer pattern */
  BAYER_MOSAIC_GBRG     = 12,	/**< Image has GBRG bayer pattern */
  BAYER_MOSAIC_GRBG     = 13,	/**< Image has GRBG bayer pattern */
  BAYER_MOSAIC_BGGR     = 14,	/**< Image has BGGR bayer pattern */
  RAW16                 = 15,	/**< Raw image, 2 bytes per pixel, format depends on camera */
  RAW8                  = 16,	/**< Raw image, 1 byte per pixel, format depends on camera */
  MONO8                 = 17,	/**< Like GRAY8 */
  MONO16                = 18,	/**< Gray-scale image, 2 bytes per pixel */
  YUV444_PACKED         = 19,	/**< Full sampled YUV, byte order Y U V */
  YVU444_PACKED         = 20,	/**< Full sampled YUV, byte order Y V U */
  YVY2                  = 21,	/**< YUV image with 4:2:2 sampling, byte order Y0 V Y1 U */
  YUV422_PLANAR_QUARTER = 22,	/**< YUV 422 image in planar format, but only quarter of the image,
				 * used for scale-conversion target, buffer is YUV422_PLANAR formatted. */
  CARTESIAN_3D_FLOAT    = 23,	/**< 3D coordinates in a packed format. Row major
                                 * (x,y,z) tuples, values as float in meters */
  CARTESIAN_3D_DOUBLE   = 24,	/**< 3D coordinates in a packed format. Row major
                                 * (x,y,z) tuples, values as double in meters */
  CARTESIAN_3D_FLOAT_RGB = 25,	/**< 3D coordinates in a packed format. Row major
                                 * (x,y,z, C) tuples, values as float in meters. C is a float
                                 * representing the color as RGB data packed as (r,g,b,I) with
                                 * r, g, b being one unsigned byte each and I is ignored. */

  RGB_PLANAR            = 26,	/**< RGB with three successive planes of R, G, and B each */
  YUV420_PLANAR         = 27,	/**< YUV 4:2:0 in planar format */
  COLORSPACE_N          = 28	/**< number of colorspaces */
} colorspace_t;


size_t           colorspace_buffer_size(colorspace_t cspace, unsigned int width, unsigned int height);
colorspace_t     colorspace_by_name(const char *colorspace);
const char *     colorspace_to_string(colorspace_t colorspace);
unsigned char *  malloc_buffer(colorspace_t colorspace, unsigned int width, unsigned int height);

} // end namespace firevision

#endif
