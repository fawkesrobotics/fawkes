
/***************************************************************************
 *  rectify.h - Header of rectification filter
 *
 *  Created: Wed Nov 07 11:07:59 2007
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

#ifndef __FIREVISION_FILTER_RECTIFY_H_
#define __FIREVISION_FILTER_RECTIFY_H_

#include <fvfilters/filter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RectificationInfoBlock;

class FilterRectify : public Filter
{
 public:
  FilterRectify(RectificationInfoBlock *rib, bool mark_zeros = true);

  virtual void apply();

 private:
  RectificationInfoBlock *__rib;
  bool __mark_zeros;
};

} // end namespace firevision

#endif

