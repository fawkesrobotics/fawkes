
/***************************************************************************
 *  colors.cpp - Skeleton Visualization GUI: colors
 *
 *  Created: Tue Mar 29 17:55:20 2011 (on the way to Magdeburg for GO2011)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/openni/utils/colors.h>

namespace fawkes {
  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

float USER_COLORS[][3] = {
  {0,1,1},
  {0,0,1},
  {0,1,0},
  {1,1,0},
  {1,0,0},
  {1,.5,0},
  {.5,1,0},
  {0,.5,1},
  {.5,0,1},
  {1,1,.5},
  {1,1,1}
};

unsigned int NUM_USER_COLORS = 10;


} // end namespace fawkes::openni
} // end namespace fawkes
