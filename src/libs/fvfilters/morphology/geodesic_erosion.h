
/***************************************************************************
 *  geodesic_erosion.h - header for morphological geodesic erosion
 *                       reconstruction
 *
 *  Created: Sat Jun 10 16:10:08 2006 (FIFA WM 2006, England vs. Paraguay)
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

#ifndef __FIREVISION_FILTERS_MORPHOLOGY_GEODESIC_EROSION_H_
#define __FIREVISION_FILTERS_MORPHOLOGY_GEODESIC_EROSION_H_

#include <fvfilters/morphology/morphologicalfilter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FilterErosion;
class FilterMax;
class ImageDiff;
class ROI;

class FilterGeodesicErosion : public MorphologicalFilter
{
 public:
  FilterGeodesicErosion(unsigned int se_size = 3);
  virtual ~FilterGeodesicErosion();

  virtual void apply();

  virtual unsigned int num_iterations();

  static const unsigned int MARKER;
  static const unsigned int MASK;

 private:
  unsigned char *isotropic_se;
  unsigned int   se_size;

  FilterErosion  *erode;
  FilterMax      *max;

  ImageDiff      *diff;

  unsigned int iterations;
};

} // end namespace firevision

#endif
