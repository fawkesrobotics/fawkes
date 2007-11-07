
/***************************************************************************
 *  geodesic_dilation.h - header for morphological geodesic dilation
 *                        reconstruction
 *
 *  Created: Wed Jun 21 16:25:22 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_FILTERS_MORPHOLOGY_GEODESIC_DILATION_H_
#define __FIREVISION_FILTERS_MORPHOLOGY_GEODESIC_DILATION_H_

#include <filters/morphology/morphologicalfilter.h>

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

#endif
