
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

#ifndef __FIREVISION_UTILS_COLOR_COLORSPACES_H_
#define __FIREVISION_UTILS_COLOR_COLORSPACES_H_

#include <sys/types.h>

typedef enum {
  CS_UNKNOWN         = 0,
  RGB,
  YUV411_PACKED,
  YUV411_PLANAR,
  YUY2,
  BGR,
  YUV422_PACKED,
  YUV422_PLANAR,
  GRAY8,
  RGB_WITH_ALPHA,
  BGR_WITH_ALPHA,
  BAYER_MOSAIC_RGGB,
  BAYER_MOSAIC_GBRG,
  BAYER_MOSAIC_GRBG,
  BAYER_MOSAIC_BGGR,
  RAW16,
  COLORSPACE_N
} colorspace_t;


inline size_t
colorspace_buffer_size(colorspace_t cspace, unsigned int width, unsigned int height)
{
  switch (cspace) {
  case RGB:
  case BGR:
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


colorspace_t     colorspace_by_name(char *colorspace);
const char *     colorspace_to_string(colorspace_t colorspace);
unsigned char *  malloc_buffer(colorspace_t colorspace, unsigned int width, unsigned int height);

#endif
