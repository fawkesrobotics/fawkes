
/***************************************************************************
 *  colorspaces.cpp - This header defines utility functions to deal with
 *                    color spaces
 *
 *  Generated: Sat Aug 12 15:26:23 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/color/colorspaces.h>

#include <cstring>
#include <cstdlib>

colorspace_t
colorspace_by_name(const char *mode)
{

  if (strcmp(mode, "RGB") == 0) {
    return RGB;
  } else if (strcmp(mode, "YUV411_PACKED") == 0) {
    return YUV411_PACKED;
  } else if (strcmp(mode, "YUV411_PLANAR") == 0) {
    return YUV411_PLANAR;
  } else if (strcmp(mode, "YUY2") == 0) {
    return YUY2;
  } else if (strcmp(mode, "YVY2") == 0) {
    return YVY2;
  } else if (strcmp(mode, "BGR") == 0) {
    return BGR;
  } else if (strcmp(mode, "YUV422_PACKED") == 0) {
    return YUV422_PACKED;
  } else if (strcmp(mode, "YUV444_PACKED") == 0) {
    return YUV444_PACKED;
  } else if (strcmp(mode, "YVU444_PACKED") == 0) {
    return YVU444_PACKED;
  } else if (strcmp(mode, "YUV422_PLANAR") == 0) {
    return YUV422_PLANAR;
  } else if (strcmp(mode, "GRAY8") == 0) {
    return GRAY8;
  } else if (strcmp(mode, "RGB_WITH_ALPHA") == 0) {
    return RGB_WITH_ALPHA;
  } else if (strcmp(mode, "BGR_WITH_ALPHA") == 0) {
    return BGR_WITH_ALPHA;
  } else if (strcmp(mode, "BAYER_MOSAIC_RGGB") == 0) {
    return BAYER_MOSAIC_RGGB;
  } else if (strcmp(mode, "BAYER_MOSAIC_GBRG") == 0) {
    return BAYER_MOSAIC_GBRG;
  } else if (strcmp(mode, "BAYER_MOSAIC_GRBG") == 0) {
    return BAYER_MOSAIC_GRBG;
  } else if (strcmp(mode, "BAYER_MOSAIC_BGGR") == 0) {
    return BAYER_MOSAIC_BGGR;
  } else if (strcmp(mode, "RAW16") == 0) {
    return RAW16;
  } else {
    return CS_UNKNOWN;
  }

}

const char *
colorspace_to_string(colorspace_t colorspace)
{
  switch (colorspace) {
  case RGB:
    return "RGB";
  case YUV411_PACKED:
    return "YUV411_PACKED";
  case YUV411_PLANAR:
    return "YUV411_PLANAR";
  case YUY2:
    return "YUY2";
  case YVY2:
    return "YVY2";
  case BGR:
    return "BGR";
  case YUV422_PACKED:
    return "YUV422_PACKED";
  case YUV444_PACKED:
    return "YUV444_PACKED";
  case YVU444_PACKED:
    return "YVU444_PACKED";
  case YUV422_PLANAR:
    return "YUV422_PLANAR";
  case GRAY8:
    return "GRAY8";
  case RGB_WITH_ALPHA:
    return "RGB_WITH_ALPHA";
  case BGR_WITH_ALPHA:
    return "BGR_WITH_ALPHA";
  case BAYER_MOSAIC_RGGB:
    return "BAYER_MOSAIC_RGGB";
  case BAYER_MOSAIC_GBRG:
    return "BAYER_MOSAIC_GBRG";
  case BAYER_MOSAIC_GRBG:
    return "BAYER_MOSAIC_GRBG";
  case BAYER_MOSAIC_BGGR:
    return "BAYER_MOSAIC_BGGR";
  case RAW16:
    return "RAW16";
  default:
    return "CS_UNKNOWN";
  }
}


unsigned char *
malloc_buffer(colorspace_t colorspace, unsigned int width, unsigned int height)
{
  return (unsigned char *)malloc(colorspace_buffer_size(colorspace, width, height));
}
