
/***************************************************************************
 *  shape_remover.cpp - Implementation of a shape remover
 *
 *  Generated: Wed Sep 28 11:26:58 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include <filters/shape_remover.h>


#include <models/shape/shapemodel.h>

/** @class FilterShapeRemover <filters/shape_remover.h>
 * Remove shapes from an image.
 */

/** Constructor. */
FilterShapeRemover::FilterShapeRemover()
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;

  shape = NULL;
}


void
FilterShapeRemover::setSrcBuffer(unsigned char *buf, ROI *roi, orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterShapeRemover::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterShapeRemover::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}


void
FilterShapeRemover::setOrientation(orientation_t ori)
{
}


const char *
FilterShapeRemover::getName()
{
  return "FilterShapeRemover";
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

  unsigned char *buffer = src_roi->getROIBufferStart( src );
  unsigned char *linestart = buffer;

  if ( (dst == NULL) || (src == dst) ) {

    for (unsigned int h = 0; h < src_roi->height; ++h) {
    
      for (unsigned int w = 0; w < src_roi->width; ++w) {
	if ((*buffer > 240) && (shape->isClose(w, h))) {
	  *buffer = 0;
	}
	buffer++;
      }
      
      linestart += src_roi->line_step;
      buffer = linestart;
    }
  } else {
    unsigned char *dst_buffer = dst_roi->getROIBufferStart( dst );
    unsigned char *dst_linestart = dst_buffer;

    for (unsigned int h = 0; h < src_roi->height; ++h) {
    
      for (unsigned int w = 0; w < src_roi->width; ++w) {
	if ((*buffer > 240) && (shape->isClose(w, h))) {
	  *dst_buffer = 0;
	} else {
	  *dst_buffer = *buffer;
	}
	buffer++;
	dst_buffer++;
      }
      
      linestart += src_roi->line_step;
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
FilterShapeRemover::setMargin( unsigned int margin )
{
  this->margin = margin;
}


/** Set shape that is to be removed.
 * @param shape shape to remove
 */
void
FilterShapeRemover::setShape( Shape *shape )
{
  this->shape = shape;
}
