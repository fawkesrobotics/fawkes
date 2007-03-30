
/***************************************************************************
 *  dilation.cpp - implementation of morphological dilation filter
 *
 *  Generated: Thu May 25 15:47:01 2006
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

#include <filters/morphology/dilation.h>

#include <fvutils/color/yuv.h>

#include <cstddef>
#include <ippi.h>

/** @class FilterDilation <filters/morphology/dilation.h>
 * Morphological dilation.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
FilterDilation::FilterDilation()
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;
  se = NULL;
}


/** Constructor with parameters.
 * @param se structuring element buffer. This is just a line-wise concatenated array
 * of values. A value of zero means ignore, any other value means to consider this
 * value.
 * @param se_width width of structuring element
 * @param se_height height of structuring element
 * @param se_anchor_x x coordinate of anchor in structuring element
 * @param se_anchor_y y coordinate of anchor in structuring element
 */
FilterDilation::FilterDilation(unsigned char *se,
			       unsigned int se_width, unsigned int se_height,
			       unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  src = dst = NULL;
  src_roi = dst_roi = NULL;

  this->se        = se;
  this->se_width  = se_width;
  this->se_height = se_height;
  this->se_anchor_x = se_anchor_x;
  this->se_anchor_y = se_anchor_y;

}


/** Destructor. */
FilterDilation::~FilterDilation()
{
}

void
FilterDilation::setSrcBuffer(unsigned char *buf, ROI *roi,
			     orientation_t ori, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterDilation::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  src = buf;
  src_roi = roi;
}


void
FilterDilation::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}


void
FilterDilation::setStructuringElement(unsigned char *se,
				      unsigned int se_width, unsigned int se_height,
				      unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  this->se          = se;
  this->se_width    = se_width;
  this->se_height   = se_height;
  this->se_anchor_x = se_anchor_x;
  this->se_anchor_y = se_anchor_y;
}


void
FilterDilation::setOrientation(orientation_t ori)
{
}


const char *
FilterDilation::getName()
{
  return "FilterDilation";
}


void
FilterDilation::apply()
{
  IppStatus status;

  if ( se == NULL ) {
    // standard 3x3 dilation

    IppiSize size;
    size.width = src_roi->width - 2;
    size.height = src_roi->height - 2;


    if ( (dst == NULL) || (dst == src) ) {
      // In-place

      // std::cout << "Running in-place with standard SE" << std::endl;

      status = ippiDilate3x3_8u_C1IR(src + ((src_roi->start.y + 1) * src_roi->line_step) + ((src_roi->start.x + 1) * src_roi->pixel_step),
				     src_roi->line_step,
				     size);
      
    } else {
      // std::cout << "Running not in-place dilation with standard SE" << std::endl;

      status = ippiDilate3x3_8u_C1R(src + ((src_roi->start.y + 1) * src_roi->line_step) + ((src_roi->start.x + 1) * src_roi->pixel_step),
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
    size.height = src_roi->height - se_width;

    IppiSize mask_size = { se_width, se_height };
    IppiPoint mask_anchor = { se_anchor_x, se_anchor_y };

    /*
    std::cout << "Dilation filter is running with the following parameters:" << std::endl
	      << "  ROI size:    " << size.width << " x " << size.height << std::endl
	      << "  mask size:   " << mask_size.width << " x " << mask_size.height << std::endl
	      << "  mask anchor: (" << mask_anchor.x  << "," << mask_anchor.y << ")" << std::endl
	      << std::endl;

    printf("  src buf:     0x%x\n", (unsigned int)src );
    printf("  dst buf:     0x%x\n", (unsigned int)dst );
    */

    if ( (dst == NULL) || (dst == src) ) {
      // In-place

      status = ippiDilate_8u_C1IR(src + ((src_roi->start.y + (se_height / 2)) * src_roi->line_step) + ((src_roi->start.x + (se_width / 2)) * src_roi->pixel_step),
				  src_roi->line_step,
				  size,
				  se, mask_size, mask_anchor);
      
    } else {
      //std::cout << "Running NOT in-place" << std::endl;

      status = ippiDilate_8u_C1R(src + ((src_roi->start.y + (se_height / 2)) * src_roi->line_step) + ((src_roi->start.x + (se_width / 2)) * src_roi->pixel_step),
				 src_roi->line_step,
				 dst + ((dst_roi->start.y + (se_height / 2)) * dst_roi->line_step) + ((dst_roi->start.x + (se_width / 2)) * dst_roi->pixel_step),
				 dst_roi->line_step,
				 size,
				 se, mask_size, mask_anchor);

      yuv422planar_copy_uv(src, dst,
			   src_roi->image_width, src_roi->image_height,
			   src_roi->start.x, src_roi->start.y,
			   src_roi->width, src_roi->height );

    }
  }

  //std::cout << "Filter exited with status " << status << std::endl;

}
