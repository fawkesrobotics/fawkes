
/***************************************************************************
 *  cvmatadapter.h - Helper to convert FireVision buffers to cv::Mat for OpenCV
 *
 *  Created: Tue May 11 15:57:58 2021
 *  Copyright  2021  Sebastian Eltester
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

#ifndef CVMATADAPTER_H
#define CVMATADAPTER_H

#include <fvutils/base/roi.h>

#include <opencv2/opencv.hpp>

namespace firevision {

class CvMatAdapter
{
public:
	static void convert_image_bgr(unsigned char *buffer, cv::Mat &image);
	static void convert_image_yuv422_planar(cv::Mat &image, unsigned char *buffer);

	//static IplImage *  create_image_from_roi(unsigned char *buffer, ROI *roi);
};

} // end namespace firevision

#endif // CVMATADAPTER_H
