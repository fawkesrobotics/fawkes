
/***************************************************************************
 *  hv_search.h - Header of horizontal- and vertical-search filter
 *
 *  Created: Tue Jul 12 14:39:27 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2006       Yuxiao Hu (Yuxiao.Hu@rwth-aachen.de)
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

#ifndef __FIREVISION_FILTER_HV_SEARCH_H_
#define __FIREVISION_FILTER_HV_SEARCH_H_

#include <fvfilters/filter.h>
#include <fvmodels/color/colormodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColorModel;

class FilterHVSearch : public Filter
{
 public:
  FilterHVSearch(ColorModel *cm, color_t what);

  virtual void apply();

 private:
  ColorModel    *cm;
  color_t        what;

};

} // end namespace firevision

#endif // __FIREVISION_FILTER_HV_SEARCH_H_

