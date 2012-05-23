
/***************************************************************************
 *  threshold.h - Header of threshold filter
 *
 *  Created: Tue Jun 07 14:25:53 2005
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

#ifndef __FIREVISION_FILTER_THRESHOLD_H_
#define __FIREVISION_FILTER_THRESHOLD_H_

#include <fvfilters/filter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FilterThreshold : public Filter
{
 public:
  FilterThreshold(unsigned char min = 128, unsigned char min_replace =   0,
		  unsigned char max = 127, unsigned char max_replace = 255);

  void set_thresholds(unsigned char min, unsigned char min_replace,
		      unsigned char max, unsigned char max_replace);

  virtual void apply();

 private:
  unsigned char max;
  unsigned char min;
  unsigned char min_replace;
  unsigned char max_replace;

};

} // end namespace firevision

#endif
