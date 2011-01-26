
/***************************************************************************
 *  hor_search.h - Header of horizontal search filter
 *
 *  Created: Wed Jul 06 11:52:27 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2005       Yuxiao Hu (Yuxiao.Hu@rwth-aachen.de)
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

#ifndef __FIREVISION_FILTERS_HOR_SEARCH_H_
#define __FIREVISION_FILTERS_HOR_SEARCH_H_

#include <fvfilters/filter.h>
#include <fvmodels/color/colormodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FilterHSearch : public Filter
{

 public:
  FilterHSearch(ColorModel *cm, color_t what);

  virtual void apply();

 private:
  ColorModel    *cm;
  color_t        what;

};

} // end namespace firevision

#endif // __FIREVISION_FILTERS_HOR_SEARCH_H_

