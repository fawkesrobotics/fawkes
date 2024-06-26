
/***************************************************************************
 *  faces.cpp - Faces classifier based on OpenCV
 *
 *  Created: Mon Dec 10 15:47:11 2007
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
#include <core/exceptions/software.h>
#include <fvclassifiers/faces.h>
#include <fvutils/adapters/cvmatadapter.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>

#include <cstddef>

namespace firevision {

/** @class FacesClassifier <fvclassifiers/faces.h>
 * Faces classifier.
 * This class provides a classifier that uses OpenCV to detect images in the given
 * image. The faces are reported back as regions of interest. Each ROI is considered
 * to contain a face.
 *
 * This code is based on the OpenCV example provided and works with the Haar cascade
 * files that come with OpenCV. The code is based on investigations by Stefan Schiffer.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param haarcascade_file Haar cascade file to use
 * @param pixel_width width of images that will be processed
 * @param pixel_height height of images that will be processed
 * @param image Optional image that is used by the classifier. If this image is NULL
 * an internal IplImage is created and the buffer converted. If you need the buffer
 * anyway pass a pointer to this image to do the conversion only once. In that case
 * the classifier assume that the image has already been converted!
 * @param haar_scale_factor Haar scale factor
 * @param min_neighbours minimum neighbours
 * @param flags flags, can only be CV_HAAR_DO_CANNY_PRUNING at the moment.
 */
FacesClassifier::FacesClassifier(const char  *haarcascade_file,
                                 unsigned int pixel_width,
                                 unsigned int pixel_height,
                                 cv::Mat     &image,
                                 float        haar_scale_factor,
                                 int          min_neighbours,
                                 int          flags)
: Classifier("FacesClassifier")
{
	haar_scale_factor_  = haar_scale_factor;
	min_neighbours_     = min_neighbours;
	flags_              = flags;
	std::string tmp_str = std::string(haarcascade_file);
	if (!cascade_.load(tmp_str)) {
		throw fawkes::Exception("Could not load Haar casca via OpenCV");
	}

	if (!image.empty()) {
		image_     = image;
		own_image_ = false;
	} else {
		image_     = cv::Mat(cv::Size(pixel_width, pixel_height), CV_8UC1, 3);
		own_image_ = true;
	}
}

/** Destructor. */
FacesClassifier::~FacesClassifier()
{
	image_.release();
}

std::list<ROI> *
FacesClassifier::classify()
{
	std::list<ROI> *rv = new std::list<ROI>();

	if (own_image_) {
		CvMatAdapter::convert_image_bgr(_src, image_);
	}

	std::vector<cv::Rect> face_seq;
	cascade_.detectMultiScale(image_, face_seq, haar_scale_factor_, min_neighbours_, flags_);

	for (int i = 0; i < int(face_seq.size()); ++i) {
		cv::Rect el = face_seq[i];
		ROI      r(el.x, el.y, el.width, el.height, _width, _height);
		r.num_hint_points = el.width * el.height;
		rv->push_back(r);
	}

	// sort, smallest first, we define num_hint_points as area enclosed by the ROI
	rv->sort();

	return rv;
}

} // end namespace firevision
