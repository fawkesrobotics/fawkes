
/***************************************************************************
 *  iplimage.cpp - Helper to convert FireVision buffers to IplImages for OpenCV
 *
 *  Created: Sat Apr 19 17:33:00 2008 (GO2008, day 1)
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/adapters/iplimage.h>

#include <fvutils/color/conversions.h>
#include <cstddef>
#include <opencv/cv.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class IplImageAdapter <fvutils/adapters/iplimage.h>
 * Adapter for OpenCV IplImages.
 * Conversion routines from FireVision buffers to OpenCV IplImages.
 * @author Tim Niemueller
 */

/** Convert image from buffer into IplImage.
 * @param buffer YUV422_PLANAR buffer of the same size as image
 * @param image IplImage the result will be written to in BGR notation
 */
void
IplImageAdapter::convert_image_bgr(unsigned char *buffer,
				   IplImage *image)
{
  convert(YUV422_PLANAR, BGR, buffer, (unsigned char *)image->imageData,
	  image->width, image->height);
}


/** Convert image from IplImage into buffer.
 * @param image IplImage with BGR notation
 * @param buffer YUV422_PLANAR of the same size to write the converted IplImage
 */
void
IplImageAdapter::convert_image_yuv422_planar(IplImage *image, 
				   unsigned char *buffer)
{
  convert(BGR, YUV422_PLANAR, (unsigned char *)image->imageData, buffer,
	  image->width, image->height);
}


/* Creates a new IplImage for a ROI.
 * This will create a new IplImage with the size of the ROI and convert the data of the
 * passed YUV422_PLANAR buffer to the IplImage.
 * @param buffer YUV422_PLANAR buffer
 * @param roi ROI to take the image from
 * @return new IplImage instance with the image from the ROI. Use cvReleaseImage after you
 * are done with it.
IplImage *
IplImageAdapter::create_image_from_roi(unsigned char *buffer, ROI *roi)
{
  IplImage *image = cvCreateImage(cvSize(roi->extent.width, roi->extent.height), IPL_DEPTH_8U, 3);

  unsigned int to_line = roi->start.y + roi->extend.height;
  unsigned char *
  for ( unsigned int h = roi->start.y; h < to_line; ++h) {
    
  }
  convert(YUV422_PLANAR, BGR, _src, (unsigned char *)__image->imageData, _width, _height);

}
 */

} // end namespace firevision
