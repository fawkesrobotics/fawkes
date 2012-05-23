
/***************************************************************************
 *  morphologicalfilter.h - interface for a morphological filter
 *
 *  Created: Thu Jun 08 09:58:21 2006
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

#ifndef __FIREVISION_FILTER_MORPHOLOGY_MORPHOLOGICAL_H_
#define __FIREVISION_FILTER_MORPHOLOGY_MORPHOLOGICAL_H_

#include <fvfilters/filter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MorphologicalFilter : public Filter {
 public:
  MorphologicalFilter(const char *name, unsigned int max_num_buffers = 1);
  virtual ~MorphologicalFilter();
  virtual void set_structuring_element(unsigned char *se,
				       unsigned int se_width,
				       unsigned int se_height,
				       unsigned int se_anchor_x,
				       unsigned int se_anchor_y );

 protected:
  /** Structuring element */
  unsigned char *se;
  /** Width of structuring element */
  unsigned int   se_width;
  /** Height of structuring element */
  unsigned int   se_height;
  /** Anchor point x offset of structuring element */
  unsigned int   se_anchor_x;
  /** Anchor point y offset of structuring element */
  unsigned int   se_anchor_y;
};

} // end namespace firevision

#endif
