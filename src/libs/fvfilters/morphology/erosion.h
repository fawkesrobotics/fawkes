
/***************************************************************************
 *  erosion.h - header for morphological erosion filter
 *
 *  Created: Fri May 26 12:12:39 2006
 *  Copyright  2005-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FILTER_MORPHOLOGY_EROSION_H_
#define __FIREVISION_FILTER_MORPHOLOGY_EROSION_H_

#include <fvfilters/morphology/morphologicalfilter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FilterErosion : public MorphologicalFilter
{
 public:
  FilterErosion();

  virtual void apply();
};

} // end namespace firevision

#endif
