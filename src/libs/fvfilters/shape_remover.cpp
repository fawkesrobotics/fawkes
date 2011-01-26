
/***************************************************************************
 *  shape_remover.cpp - Implementation of a shape remover
 *
 *  Created: Wed Sep 28 11:26:58 2005
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

#include <fvfilters/shape_remover.h>


#include <fvmodels/shape/shapemodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FilterShapeRemover <fvfilters/shape_remover.h>
 * Remove shapes from an image.
 */

/** Constructor. */
FilterShapeRemover::FilterShapeRemover()
  : Filter("FilterShapeRemover")
{
  shape = NULL;
}


void
FilterShapeRemover::apply()
{
  /* Code to remove lines here
   * INPUT:  Pre-filtered image or part of image that has clear edges set (white
   *         value at edge > 240)
   * OUTPUT: the edges close to the given shape have been removed
   */

  if (shape == NULL) return;

  shape->setMargin( margin );

  unsigned char *buffer = src_roi[0]->get_roi_buffer_start( src[0] );
  unsigned char *linestart = buffer;

  if ( (dst == NULL) || (src[0] == dst) ) {

    for (unsigned int h = 0; h < src_roi[0]->height; ++h) {
    
      for (unsigned int w = 0; w < src_roi[0]->width; ++w) {
	if ((*buffer > 240) && (shape->isClose(w, h))) {
	  *buffer = 0;
	}
	buffer++;
      }
      
      linestart += src_roi[0]->line_step;
      buffer = linestart;
    }
  } else {
    unsigned char *dst_buffer = dst_roi->get_roi_buffer_start( dst );
    unsigned char *dst_linestart = dst_buffer;

    for (unsigned int h = 0; h < src_roi[0]->height; ++h) {
    
      for (unsigned int w = 0; w < src_roi[0]->width; ++w) {
	if ((*buffer > 240) && (shape->isClose(w, h))) {
	  *dst_buffer = 0;
	} else {
	  *dst_buffer = *buffer;
	}
	buffer++;
	dst_buffer++;
      }
      
      linestart += src_roi[0]->line_step;
      dst_linestart += dst_roi->line_step;
      buffer = linestart;
      dst_buffer = dst_linestart;
    }
  }
}


/** Set margin.
 * @param margin margin around shape to be close to a point.
 */
void
FilterShapeRemover::set_margin( unsigned int margin )
{
  this->margin = margin;
}


/** Set shape that is to be removed.
 * @param shape shape to remove
 */
void
FilterShapeRemover::set_shape( Shape *shape )
{
  this->shape = shape;
}

} // end namespace firevision
