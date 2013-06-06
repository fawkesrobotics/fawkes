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

#include "threasholds_black.h"

namespace firevision
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColorModelThresholds <fvmodels/color/thresholds.h>
 * Really simple thresholds-based model with variable thresholds. 
 * just for initial development of color models.
 */

color_t
ColorModelBlack::determine(unsigned int y,
				unsigned int u,
				unsigned int v) const
{
	if ( y <= this->thrashold 
		&& u >= 90 && u <= 150 
		&& v >= 90 && v <= 150) {
		return C_BLACK;
	} else {
		return C_OTHER;
	}
}

const char *
ColorModelBlack::get_name()
{
	return "ColorModelBlack";
}

} // end namespace firevision
