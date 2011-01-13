
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

#ifndef __FIREVISION_COLORMODEL_THRESHOLDS_H_
#define __FIREVISION_COLORMODEL_THRESHOLDS_H_

#include <fvmodels/color/colormodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/* The following thresholds define certain color regions in the
   two-dimensional UV-Colorspace (ignoring the Y-component).
   It is assumed that the Y-, U- and V-components range from 0 to 255 each,
   and that (0, 0) is at the lower left corner. */

// 1/8 * 255 = 31
#define THRESHOLD_ORANGE_U_LOW     0

// 3/8 * 255 = 94
#define THRESHOLD_ORANGE_U_HIGH   120

// 3/4 * 255 = 191
#define THRESHOLD_ORANGE_V_LOW    170

// 5/8 * 255
#define THRESHOLD_MAGENTA_U_LOW   159

// 5/8 * 255
#define THRESHOLD_MAGENTA_V_LOW   159

//1/4 * 255
#define THRESHOLD_CYAN_U_LOW       63

// 5/8 * 255
#define THRESHOLD_CYAN_U_HIGH     159

//1/4 * 255
#define THRESHOLD_CYAN_V_HIGH      63

// 3/4 * 255
#define THRESHOLD_BLUE_U_LOW      191

// 0 :-)
#define THRESHOLD_BLUE_V_HIGH      90

// 1/8 * 255
#define THRESHOLD_YELLOW_U_HIGH    31

// 3/4 * 255
#define THRESHOLD_YELLOW_V_LOW    191

//1/4 * 255
#define THRESHOLD_GREEN_U_HIGH     63

// 5/8 * 255
#define THRESHOLD_GREEN_V_HIGH    159

// 2/3 * 255
#define THRESHOLD_WHITE_Y_LOW     170


class ColorModelThresholds : public ColorModel
{
 public:

  color_t       determine(unsigned int y,
			  unsigned int u,
			  unsigned int v) const;

  const char *  get_name();
  void          print_thresholds();

};

} // end namespace firevision

#endif
