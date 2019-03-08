
/***************************************************************************
 *  conversions.h - Conversions
 *
 *  Created: Sat Aug 12 15:19:31 2006
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

#ifndef FIREVISION_UTILS_COLOR_CONVERSIONS_H_
#define FIREVISION_UTILS_COLOR_CONVERSIONS_H_

#include <fvutils/color/colorspaces.h>

namespace firevision {

extern void convert(colorspace_t         from,
                    colorspace_t         to,
                    const unsigned char *src,
                    unsigned char *      dst,
                    unsigned int         width,
                    unsigned int         height);

extern void grayscale(colorspace_t   cspace,
                      unsigned char *src,
                      unsigned char *dst,
                      unsigned int   width,
                      unsigned int   height);

} // end namespace firevision

#endif
