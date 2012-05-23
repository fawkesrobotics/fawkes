
/***************************************************************************
 *  or.h - Header of Or filter
 *
 *  Created: Fri May 23 14:24:57 2005
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

#if ! (defined(HAVE_IPP) || defined(HAVE_OPENCV))
#error "Neither IPP nor OpenCV installed"
#endif

#ifndef __FIREVISION_FILTER_OR_H_
#define __FIREVISION_FILTER_OR_H_

#include <fvfilters/filter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FilterOr : public Filter
{
 public:
  FilterOr();

  virtual void apply();
};

} // end namespace firevision

#endif
