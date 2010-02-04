
/***************************************************************************
 *  rectinfo.cpp - Rectification info file definitions
 *
 *  Created: Tue Nov 27 02:01:06 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/rectification/rectinfo.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

const char * rectinfo_camera_strings[] =
  {
    "Main",
    "Left",
    "Right",
    "Center",
    "Top",
    0
  };


const char * rectinfo_type_strings[] =
  {
    "Invalid format",
    "Rectification LUT 16x16",
    0
  };

} // end namespace firevision
