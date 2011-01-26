
/***************************************************************************
 *  nothing.h - Header of a filter that does nothing. Wonderful, isn't it.
 *              Used in FilterFactory to avoid NULL checks
 *
 *  Created: Fri Jun 17 13:28:21 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FILTER_NOTHING_H_
#define __FIREVISION_FILTER_NOTHING_H_

#include <fvfilters/filter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterNothing <fvfilters/nothing.h>
 * A filter that does... nothing.
 * @author Tim Niemueller
 */

class FilterNothing : public Filter
{
 public:
  /** Constructor. */
  FilterNothing() : Filter("FilterNothing") {}

  virtual void apply() {}
};

} // end namespace firevision

#endif
