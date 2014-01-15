
/***************************************************************************
 *  colorspaces.cpp - This header defines utility functions to deal with
 *                    color spaces
 *
 *  Generated: Sat Aug 12 15:26:23 2006
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

#include <fvutils/color/colorspaces.h>

#include <cstring>
#include <cstdlib>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

colorspace_t
colorspace_by_name(const char *mode)
{

  if (strcmp(mode, "RGB") == 0) {
    return RGB;
  } else if (strcmp(mode, "YUV411_PACKED") == 0) {
    return YUV411_PACKED;
  } else if (strcmp(mode, "YUV411_PLANAR") == 0) {
    return YUV411_PLANAR;
  } else if (strcmp(mode, "YUV420_PLANAR") == 0) {
    return YUV420_PLANAR;
  } else if (strcmp(mode, "YUY2") == 0) {
    return YUY2;
  } else if (strcmp(mode, "YVY2") == 0) {
    return YVY2;
  } else if (strcmp(mode, "RGB_PLANAR") == 0) {
    return RGB_PLANAR;
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
  } else if (strcmp(mode, "YUV422_PLANAR_QUARTER") == 0) {
    return YUV422_PLANAR_QUARTER;
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
  } else if (strcmp(mode, "CARTESIAN_3D_FLOAT") == 0) {
    return CARTESIAN_3D_FLOAT;
  } else if (strcmp(mode, "CARTESIAN_3D_DOUBLE") == 0) {
    return CARTESIAN_3D_DOUBLE;
  } else if (strcmp(mode, "CARTESIAN_3D_FLOAT_RGB") == 0) {
    return CARTESIAN_3D_FLOAT_RGB;
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
  case YUV420_PLANAR:
    return "YUV420_PLANAR";
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
  case YUV422_PLANAR_QUARTER:
    return "YUV422_PLANAR_QUARTER";
  case GRAY8:
    return "GRAY8";
  case MONO8:
    return "MONO8";
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
  case CARTESIAN_3D_FLOAT:
    return "CARTESIAN_3D_FLOAT";
  case CARTESIAN_3D_DOUBLE:
    return "CARTESIAN_3D_DOUBLE";
  case CARTESIAN_3D_FLOAT_RGB:
    return "CARTESIAN_3D_FLOAT_RGB";
  case RGB_PLANAR:
    return "RGB_PLANAR";
  default:
    return "CS_UNKNOWN";
  }
}


unsigned char *
malloc_buffer(colorspace_t colorspace, unsigned int width, unsigned int height)
{
  return (unsigned char *)malloc(colorspace_buffer_size(colorspace, width, height));
}

size_t
colorspace_buffer_size(colorspace_t cspace, unsigned int width, unsigned int height)
{
  switch (cspace) {
  case RGB:
  case BGR:
  case RGB_PLANAR:
  case YUV444_PACKED:
  case YVU444_PACKED:
    return (width * height * 3);
 
  case RGB_WITH_ALPHA:
  case BGR_WITH_ALPHA:
    return (width * height * 4);

  case YUV411_PACKED:
  case YUV411_PLANAR:
  case YUV420_PLANAR:
    return (width * height * 3 / 2);
    
  case YUY2:
  case YVY2:
  case YUV422_PACKED:
  case YUV422_PLANAR:
    return (width * height * 2);

  case MONO16:
  case RAW16:
    return (width * height * 2);
    
  case RAW8:
  case GRAY8:
  case MONO8:
  case BAYER_MOSAIC_RGGB:
  case BAYER_MOSAIC_GBRG:
  case BAYER_MOSAIC_GRBG:
  case BAYER_MOSAIC_BGGR:
    return (width * height);

  case YUV422_PLANAR_QUARTER:
    return (width * height) / 2;

  case CARTESIAN_3D_FLOAT:
    return (3 * width * height * sizeof(float));

  case CARTESIAN_3D_DOUBLE:
    return (3 * width * height * sizeof(double));

  case CARTESIAN_3D_FLOAT_RGB:
    return (4 * width * height * sizeof(float));

  case CS_UNKNOWN:
  case COLORSPACE_N:
    return 0;
  }

  return 0;
}

} // end namespace firevision
