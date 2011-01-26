
/***************************************************************************
 *  geodesic_erosion.cpp - implementation of morphological geodesic erosion
 *
 *  Created: Sat Jun 10 16:21:30 2006
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

#include <core/exception.h>

#include <fvfilters/morphology/geodesic_erosion.h>
#include <fvfilters/morphology/segenerator.h>
#include <fvfilters/morphology/erosion.h>
#include <fvfilters/max.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/statistical/imagediff.h>

#include <cstdlib>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Marker */
const unsigned int FilterGeodesicErosion::MARKER = 0;
/** Mask */
const unsigned int FilterGeodesicErosion::MASK   = 1;

#define ERROR(m) {							\
    fawkes::Exception e("FilterGeodesicErosion failed");			\
    e.append("Function: %s", __FUNCTION__);				\
    e.append("Message:  %s", m);					\
    throw e;								\
  }

/** @class FilterGeodesicErosion <fvfilters/morphology/geodesic_erosion.h>
 * Morphological geodesic erosion.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param se_size Structuring element size.
 */
FilterGeodesicErosion::FilterGeodesicErosion(unsigned int se_size)
  : MorphologicalFilter("Morphological Geodesic Erosion")
{
  this->se_size  = (se_size > 0) ? se_size : 1;
  iterations = 0;

  erode  = new FilterErosion();
  max    = new FilterMax();
  diff   = new ImageDiff();

  isotropic_se = SEGenerator::square(this->se_size, this->se_size);

  erode->set_structuring_element( isotropic_se, se_size, se_size, se_size / 2, se_size / 2 );
}


/** Destructor. */
FilterGeodesicErosion::~FilterGeodesicErosion()
{
  delete erode;
  delete max;
  delete diff;
  free( isotropic_se );
}


void
FilterGeodesicErosion::apply()
{
  if ( dst == NULL ) ERROR("dst == NULL");
  if ( src[MASK] == NULL ) ERROR("src[MASK] == NULL");
  if ( src[MARKER] == NULL ) ERROR("src[MARKER] == NULL");
  if ( *(src_roi[MASK]) != *(src_roi[MARKER]) ) ERROR("marker and mask ROI differ");

  unsigned char *tmp = (unsigned char *)malloc(colorspace_buffer_size(YUV422_PLANAR, src_roi[MARKER]->image_width, src_roi[MARKER]->image_height) );
  memcpy( tmp, src[MARKER], colorspace_buffer_size(YUV422_PLANAR, src_roi[MARKER]->image_width, src_roi[MARKER]->image_height) );

  diff->setBufferA( tmp, src_roi[MARKER]->image_width, src_roi[MARKER]->image_height );
  diff->setBufferB( dst, dst_roi->image_width, dst_roi->image_height );

  erode->set_src_buffer( tmp, src_roi[MARKER] );

  max->set_src_buffer( src[MASK], src_roi[MASK], 0 );
  max->set_src_buffer( tmp, src_roi[MARKER], 1 );
  max->set_dst_buffer( tmp, src_roi[MARKER] );


  iterations = 0;
  do {
    memcpy(dst, tmp, colorspace_buffer_size(YUV422_PLANAR, dst_roi->image_width, dst_roi->image_height) );
    erode->apply();
    max->apply();
  } while (diff->different() && ( ++iterations < 255) );

  // std::cout << i << " iterations done for geodesic erosion" << std::endl;


  free( tmp );

}


/** Get the number of iterations.
 * @return the number of iterations that were necessary to get a stable result in the
 * last call to apply().
 */
unsigned int
FilterGeodesicErosion::num_iterations()
{
  return iterations;
}

} // end namespace firevision
