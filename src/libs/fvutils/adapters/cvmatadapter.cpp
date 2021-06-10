

/***************************************************************************
 *  cvmatadapter.cpp - Helper to convert FireVision buffers to cv::Mat for OpenCV
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

#include <fvutils/adapters/cvmatadapter.h>
#include <fvutils/color/conversions.h>

#include <cstddef>
#include <opencv2/opencv.hpp>

namespace firevision {

/** @class CvMatAdapter <fvutils/adapters/cvmatadapter.h>
 * Adapter for OpenCV Mat.
 * Conversion routines from FireVision buffers to OpenCV Mat.
 * @author Sebastian Eltester
 */

/** Convert image from buffer into cv::Mat.
 * @param buffer YUV422_PLANAR buffer of the same size as image
 * @param image cv::Mat the result will be written to in BGR notation
 */
void
CvMatAdapter::convert_image_bgr(unsigned char *buffer, cv::Mat &image)
{
	int buffer_size = 0;
	while (buffer[buffer_size] != '\0') {
		buffer_size++;
	}
	unsigned char tmp[buffer_size];
	convert(YUV422_PLANAR, YUV422_PACKED, buffer, tmp, image.cols, image.rows);
	cv::Mat tmp_mat = cv::Mat(image.size(), CV_8UC2, tmp);
	cv::cvtColor(tmp_mat, image, cv::COLOR_YUV2BGR_UYVY, 3);
}

/** Convert image from cv::Mat into buffer.
 * @param image cv::Mat with BGR notation
 * @param buffer YUV422_PLANAR of the same size to write the converted cv::Mat
 */
void
CvMatAdapter::convert_image_yuv422_planar(cv::Mat &image, unsigned char *buffer)
{
	cv::Mat tmp_mat = cv::Mat(image);
	cv::cvtColor(image, tmp_mat, cv::COLOR_BGR2YUV, 3);
	convert(YUV422_PACKED, YUV422_PLANAR, tmp_mat.data, buffer, image.cols, image.rows);
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
  convert(YUV422_PLANAR, BGR, _src, (unsigned char *)image_->imageData, _width, _height);

}
 */

} // end namespace firevision
