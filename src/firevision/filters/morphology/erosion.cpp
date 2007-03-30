
/***************************************************************************
 *  erosion.cpp - implementation of morphological erosion filter
 *
 *  Generated: Fri May 26 12:13:22 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <filters/morphology/erosion.h>

#include <fvutils/color/yuv.h>

#include <cstddef>
#include <ippi.h>

/** @class FilterErosion <filters/morphology/erosion.h>
 * Morphological erosion.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
FilterErosion::FilterErosion()
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;
  se = NULL;
}


void
FilterErosion::setSrcBuffer(unsigned char *buf, ROI *roi,
			     orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterErosion::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterErosion::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}


void
FilterErosion::setStructuringElement(unsigned char *se,
				     unsigned int se_width, unsigned int se_height,
				     unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  this->se        = se;
  this->se_width  = se_width;
  this->se_height = se_height;
  this->se_anchor_x = se_anchor_x;
  this->se_anchor_y = se_anchor_y;
}


void
FilterErosion::setOrientation(orientation_t ori)
{
}


const char *
FilterErosion::getName()
{
  return "FilterErosion";
}


void
FilterErosion::apply()
{
  IppStatus status;

  if ( se == NULL ) {
    // standard 3x3 erosion

    IppiSize size;
    size.width = src_roi->width - 2;
    size.height = src_roi->height - 2;


    if ( (dst == NULL) || (dst == src) ) {
      // In-place
      status = ippiErode3x3_8u_C1IR(src + ((src_roi->start.y + 1) * src_roi->line_step) + ((src_roi->start.x + 1) * src_roi->pixel_step),
				    src_roi->line_step,
				    size);
      
    } else {
      status = ippiErode3x3_8u_C1R(src + ((src_roi->start.y + 1) * src_roi->line_step) + ((src_roi->start.x + 1) * src_roi->pixel_step),
				   src_roi->line_step,
				   dst + ((dst_roi->start.y + 1) * dst_roi->line_step) + ((dst_roi->start.x + 1) * dst_roi->pixel_step),
				   dst_roi->line_step,
				   size);

      yuv422planar_copy_uv(src, dst,
			   src_roi->image_width, src_roi->image_height,
			   src_roi->start.x, src_roi->start.y,
			   src_roi->width, src_roi->height );
    }
  } else {
    // we have a custom SE

    IppiSize size;
    size.width = src_roi->width - se_width;
    size.height = src_roi->height - se_height;

    IppiSize mask_size = { se_width, se_height };
    IppiPoint mask_anchor = { se_anchor_x, se_anchor_y };

    if ( (dst == NULL) || (dst == src) ) {
      // In-place
      status = ippiErode_8u_C1IR(src + ((src_roi->start.y + (se_height / 2)) * src_roi->line_step) + ((src_roi->start.x + (se_width / 2)) * src_roi->pixel_step),
				 src_roi->line_step,
				 size,
				 se, mask_size, mask_anchor);

      //std::cout << "in-place operation ended with status " << status << std::endl;
      
    } else {
      status = ippiErode_8u_C1R(src + ((src_roi->start.y + (se_height / 2)) * src_roi->line_step) + ((src_roi->start.x + (se_width / 2)) * src_roi->pixel_step),
				src_roi->line_step,
				dst + ((dst_roi->start.y + (se_height / 2)) * dst_roi->line_step) + ((dst_roi->start.x + (se_width / 2)) * dst_roi->pixel_step),
				dst_roi->line_step,
				size,
				se, mask_size, mask_anchor);

      // std::cout << "NOT in-place operation ended with status " << status << std::endl;

      yuv422planar_copy_uv(src, dst,
			   src_roi->image_width, src_roi->image_height,
			   src_roi->start.x, src_roi->start.y,
			   src_roi->width, src_roi->height );
    }

  }

}
