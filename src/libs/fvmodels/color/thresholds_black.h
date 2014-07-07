
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

#ifndef __FIREVISION_DARKNESS_THRESHOLD_H_
#define __FIREVISION_DARKNESS_THRESHOLD_H_

#include <fvmodels/color/colormodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColorModelBlack : public ColorModel
{
 private:
  unsigned int y_thresh_, u_thresh_, v_thresh_;
  unsigned int ref_u_, ref_v_;
 public:

  ColorModelBlack(unsigned int y_thresh = 30, unsigned int u_thresh = 30, unsigned int v_thresh = 30,
    unsigned int ref_u = 128, unsigned int ref_v = 128);

  color_t       determine(unsigned int y,
			  unsigned int u,
			  unsigned int v) const;

  const char *  get_name();

};

} // end namespace firevision

#endif
