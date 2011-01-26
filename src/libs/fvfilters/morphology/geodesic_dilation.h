
/***************************************************************************
 *  geodesic_dilation.h - header for morphological geodesic dilation
 *                        reconstruction
 *
 *  Created: Wed Jun 21 16:25:22 2006
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

#ifndef __FIREVISION_FILTERS_MORPHOLOGY_GEODESIC_DILATION_H_
#define __FIREVISION_FILTERS_MORPHOLOGY_GEODESIC_DILATION_H_

#include <fvfilters/morphology/morphologicalfilter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FilterDilation;
class FilterMin;
class ImageDiff;
class ROI;

class FilterGeodesicDilation : public MorphologicalFilter
{
 public:
  FilterGeodesicDilation(unsigned int se_size = 3);
  virtual ~FilterGeodesicDilation();

  virtual void apply();

  virtual unsigned int num_iterations();

  static const unsigned int MARKER;
  static const unsigned int MASK;

 private:
  unsigned char *isotropic_se;
  unsigned int   se_size;

  FilterDilation *dilate;
  FilterMin      *min;

  ImageDiff      *diff;

  unsigned int    iterations;

};

} // end namespace firevision

#endif
