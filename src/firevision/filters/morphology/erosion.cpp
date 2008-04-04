
/***************************************************************************
 *  erosion.cpp - implementation of morphological erosion filter
 *
 *  Created: Fri May 26 12:13:22 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#include <filters/morphology/erosion.h>

#include <fvutils/color/yuv.h>
#include <core/exception.h>

#include <cstddef>
#include <ippi.h>

/** @class FilterErosion <filters/morphology/erosion.h>
 * Morphological erosion.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
FilterErosion::FilterErosion()
  : MorphologicalFilter("Morphological Erosion")
{
}


void
FilterErosion::apply()
{
  IppStatus status;

  if ( se == NULL ) {
    // standard 3x3 erosion

    IppiSize size;
    size.width = src_roi[0]->width - 2;
    size.height = src_roi[0]->height - 2;


    if ( (dst == NULL) || (dst == src[0]) ) {
      // In-place
      status = ippiErode3x3_8u_C1IR(src[0] + ((src_roi[0]->start.y + 1) * src_roi[0]->line_step) + ((src_roi[0]->start.x + 1) * src_roi[0]->pixel_step),
				    src_roi[0]->line_step,
				    size);
      
    } else {
      status = ippiErode3x3_8u_C1R(src[0] + ((src_roi[0]->start.y + 1) * src_roi[0]->line_step) + ((src_roi[0]->start.x + 1) * src_roi[0]->pixel_step),
				   src_roi[0]->line_step,
				   dst + ((dst_roi->start.y + 1) * dst_roi->line_step) + ((dst_roi->start.x + 1) * dst_roi->pixel_step),
				   dst_roi->line_step,
				   size);

      yuv422planar_copy_uv(src[0], dst,
			   src_roi[0]->image_width, src_roi[0]->image_height,
			   src_roi[0]->start.x, src_roi[0]->start.y,
			   src_roi[0]->width, src_roi[0]->height );
    }
  } else {
    // we have a custom SE

    IppiSize size;
    size.width = src_roi[0]->width - se_width;
    size.height = src_roi[0]->height - se_height;

    IppiSize mask_size = { se_width, se_height };
    IppiPoint mask_anchor = { se_anchor_x, se_anchor_y };

    if ( (dst == NULL) || (dst == src[0]) ) {
      // In-place
      status = ippiErode_8u_C1IR(src[0] + ((src_roi[0]->start.y + (se_height / 2)) * src_roi[0]->line_step) + ((src_roi[0]->start.x + (se_width / 2)) * src_roi[0]->pixel_step),
				 src_roi[0]->line_step,
				 size,
				 se, mask_size, mask_anchor);

      //std::cout << "in-place operation ended with status " << status << std::endl;
      
    } else {
      status = ippiErode_8u_C1R(src[0] + ((src_roi[0]->start.y + (se_height / 2)) * src_roi[0]->line_step) + ((src_roi[0]->start.x + (se_width / 2)) * src_roi[0]->pixel_step),
				src_roi[0]->line_step,
				dst + ((dst_roi->start.y + (se_height / 2)) * dst_roi->line_step) + ((dst_roi->start.x + (se_width / 2)) * dst_roi->pixel_step),
				dst_roi->line_step,
				size,
				se, mask_size, mask_anchor);

      // std::cout << "NOT in-place operation ended with status " << status << std::endl;

      yuv422planar_copy_uv(src[0], dst,
			   src_roi[0]->image_width, src_roi[0]->image_height,
			   src_roi[0]->start.x, src_roi[0]->start.y,
			   src_roi[0]->width, src_roi[0]->height );
    }

  }

  if ( status != ippStsNoErr ) {
    throw Exception("Morphological erosion failed with %i\n", status);
  }

}
