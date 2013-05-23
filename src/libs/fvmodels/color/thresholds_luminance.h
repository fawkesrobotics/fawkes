
/**************************************************************************
 *  thresholds.h - This header defines thresholds for color classification
 *
 *  Created: Wed May 11 11:22:00 2005
 *  Copyright  2005  Martin Heracles <Martin.Heracles@rwth-aachen.de>
 *                   Tim Niemueller  [www.niemueller.de]
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_COLORMODEL_LUMINANCE_H_
#define __FIREVISION_COLORMODEL_LUMINANCE_H_

#include <fvmodels/color/colormodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/* The following thresholds define certain color regions in the
   two-dimensional UV-Colorspace (ignoring the Y-component).
   It is assumed that the Y-, U- and V-components range from 0 to 255 each,
   and that (0, 0) is at the lower left corner. */

// 2/3 * 255
#define THRESHOLD_WHITE_Y_LOW     170


class ColorModelLuminance : public ColorModel
{
 public:
  ColorModelLuminance(const unsigned int threshold_white_low = THRESHOLD_WHITE_Y_LOW);

  color_t       determine(unsigned int y,
			  unsigned int u,
			  unsigned int v) const;

  const char *  get_name();
  void          print_thresholds();
 private:
  unsigned int threshold_white_low_;

};

} // end namespace firevision

#endif
