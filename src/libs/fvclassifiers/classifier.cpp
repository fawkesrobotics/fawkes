
/***************************************************************************
 *  classifier.cpp - Abstract class defining a classifier
 *
 *  Created: Mon Dec 10 11:35:36 2007
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

#include <fvclassifiers/classifier.h>
#include <cstring>
#include <cstdlib>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Classifier <fvclassifiers/classifier.h>
 * Classifier to extract regions of interest.
 * The classifier finds regions of interest (ROI) by some a priori knowledge
 * like known colors or shapes. The list of ROIs returned by classify() _must_
 * be disjunct, meaning that no ROIs overlap each other.
 * Do appropriate merging or shrinking of the ROIs. See the ReallySimpleClassifier
 * for an example.
 * @author Tim Niemueller
 *
 * @fn std::list< ROI > * Classifier::classify() = 0
 * Classify image.
 * The current buffer is processed and scanned for the features the classifier
 * has been written and initialized for. It returns a list of disjunct regions
 * of interest.
 * @return disjunct list of extracted regions of interest
 */


/** Constructor.
 * @param name classifier name
 */
Classifier::Classifier(const char *name)
{
  __name  = strdup(name);
  _src    = NULL;
  _width  = 0;
  _height = 0;
}


/** Destructor. */
Classifier::~Classifier()
{
  free(__name);
}


/** Set source buffer.
 * @param yuv422_planar a YUV422 planar buffer with the source image to
 * classify. The classifier may NOT modify the image in any way. If that is
 * required the classifier shall make a copy of the image.
 * @param width width of buffer in pixels
 * @param height height of buffer in pixels
 */
void
Classifier::set_src_buffer(unsigned char *yuv422_planar, unsigned int width,
			   unsigned int height)
{
  _src    = yuv422_planar;
  _width  = width;
  _height = height;
}


/** Get name of classifier.
 * @return name of classifier.
 */
const char *
Classifier::name() const
{
  return __name;
}

} // end namespace firevision
