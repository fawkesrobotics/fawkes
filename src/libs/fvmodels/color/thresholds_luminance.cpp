
/***************************************************************************
 *  thresholds.cpp - Implementation of a thresholds color model
 *
 *  Created: Wed May 18 13:59:18 2005
 *  Copyright  2005  Tim Niemueller  [www.niemueller.de]
 *                   Matrin Heracles <martin.heracles@rwth-aachen.de>
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

#include <iostream>

#include <fvmodels/color/thresholds_luminance.h>

using namespace std;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColorModelLuminance <fvmodels/color/thresholds_luminance.h>
 * Really simple thresholds-based model with some hard-coded thresholds. Was
 * just for initial development of color models.
 */

/** Constructor.
 * @param threshold_white_low minimum luminance value to mark color
 */
ColorModelLuminance::ColorModelLuminance(const unsigned int threshold_white_low)
{
  threshold_white_low_ = threshold_white_low;
}

color_t
ColorModelLuminance::determine(unsigned int y,
				unsigned int u,
				unsigned int v) const
{
  if ( y >= threshold_white_low_) {
    return C_WHITE;
  }
  else {
    return C_OTHER;
  }
}

const char *
ColorModelLuminance::get_name()
{
  return "ColorModelLuminance";
}


/** Print the thresholds to stdout.
 */
void
ColorModelLuminance::print_thresholds()
{
  cout << "ColorModelLuminance" << endl
       << "==========================================================" << endl
       << "White:  y_low=" << threshold_white_low_
       << endl;
}

} // end namespace firevision
