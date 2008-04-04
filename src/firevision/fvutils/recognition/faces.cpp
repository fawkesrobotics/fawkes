
/***************************************************************************
 *  faces.cpp - Face Recognizer
 *
 *  Created: Wed Dec 12 13:08:42 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/recognition/faces.h>

/** @class FaceRecognizer <fvutils/recognition/faces.h>
 * Face Recognizer.
 * This class takes an image and a number of ROIs as input. The ROIs are then
 * scanned and a map with labeled ROIs is returned.
 * @author Tim Niemueller
 */

/** Constructor. */
FaceRecognizer::FaceRecognizer()
{
}


/** Destructor. */
FaceRecognizer::~FaceRecognizer()
{
}


/** Recognize faces.
 * Scans the given ROIs in the image for faces and returns a map of labeled
 * faces.
 * @param buffer image buffer
 * @param rois reference to list of ROIs, can be the list of ROIs returned by a
 * classifier like the FacesClassifier.
 * @return map of ROIs with strings as labels for the ROI.
 */
FaceRecognizer::FaceRoiMap *
FaceRecognizer::recognize(unsigned char *buffer, std::list<ROI> &rois)
{
  FaceRoiMap *rv = new FaceRoiMap();

  return rv;
}
