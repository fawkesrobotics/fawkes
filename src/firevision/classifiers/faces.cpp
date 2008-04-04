
/***************************************************************************
 *  faces.cpp - Faces classifier based on OpenCV
 *
 *  Created: Mon Dec 10 15:47:11 2007
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

#include <classifiers/faces.h>

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>

#include <opencv/cv.h>

/** @class FacesClassifier <classifiers/faces.h>
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
 * @param haar_scale_factor Haar scale factor
 * @param min_neighbours minimum neighbours
 * @param flags flags, can only be CV_HAAR_DO_CANNY_PRUNING at the moment.
 */
FacesClassifier::FacesClassifier(const char *haarcascade_file,
				 unsigned int pixel_width, unsigned int pixel_height,
				 float haar_scale_factor, int min_neighbours, int flags)
  : Classifier("FacesClassifier")
{
  __haar_scale_factor = haar_scale_factor;
  __min_neighbours = min_neighbours;
  __flags = flags;

  __cascade = (CvHaarClassifierCascade *) cvLoad(haarcascade_file);
  if ( ! __cascade ) {
    throw Exception("Could not load Haar casca via OpenCV");
  }

  __storage = cvCreateMemStorage(0);
  if ( ! __storage ) {
    cvReleaseHaarClassifierCascade(&__cascade);
    throw Exception("Could not initialize OpenCV memory");
  }

  __image = cvCreateImage(cvSize(pixel_width, pixel_height), IPL_DEPTH_8U, 3);
}


/** Destructor. */
FacesClassifier::~FacesClassifier()
{
  cvReleaseHaarClassifierCascade(&__cascade);
  cvReleaseMemStorage(&__storage);
  cvReleaseImage(&__image);
}


std::list< ROI > *
FacesClassifier::classify()
{
  std::list< ROI > *rv = new std::list< ROI >();

  convert(YUV422_PLANAR, BGR, _src, (unsigned char *)__image->imageData, _width, _height);

  CvSeq *face_seq = cvHaarDetectObjects(__image, __cascade, __storage,
					__haar_scale_factor, __min_neighbours, __flags);

  for ( int i = 0; i < face_seq->total; ++i) {
    CvAvgComp el = *(CvAvgComp*)cvGetSeqElem(face_seq, i);
    ROI r(el.rect.x, el.rect.y, el.rect.width, el.rect.height, _width, _height);
    rv->push_back(r);
  }

  return rv;
}
