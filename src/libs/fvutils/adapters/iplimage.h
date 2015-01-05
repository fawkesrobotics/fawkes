
/***************************************************************************
 *  iplimage.h - Helper to convert FireVision buffers to IplImages for OpenCV
 *
 *  Created: Sat Apr 19 17:28:58 2008 (GO2008, day 1)
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

#ifndef __FIREVISION_FVUTILS_ADAPTERS_IPLIMAGE_H_
#define __FIREVISION_FVUTILS_ADAPTERS_IPLIMAGE_H_

#include <fvutils/base/roi.h>

typedef struct _IplImage IplImage;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class IplImageAdapter
{
 public:
  static void convert_image_bgr(unsigned char *buffer, IplImage *image);
  static void convert_image_yuv422_planar(IplImage *image, unsigned char *buffer);

  //static IplImage *  create_image_from_roi(unsigned char *buffer, ROI *roi);

};

} // end namespace firevision

#endif
