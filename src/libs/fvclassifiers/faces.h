
/***************************************************************************
 *  faces.h - Faces classifier based on OpenCV
 *
 *  Created: Mon Dec 10 15:46:06 2007
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

#ifndef _FIREVISION_CLASSIFIERS_FACES_H_
#define _FIREVISION_CLASSIFIERS_FACES_H_

#include <fvclassifiers/classifier.h>

struct CvHaarClassifierCascade;
struct CvMemStorage;
typedef struct _IplImage IplImage;

namespace firevision {

class FacesClassifier : public Classifier
{
public:
	FacesClassifier(const char * haarcascade_file,
	                unsigned int pixel_width,
	                unsigned int pixel_height,
	                IplImage *   image             = 0,
	                float        haar_scale_factor = 1.1,
	                int          min_neighbours    = 3,
	                int          flags             = 0);

	virtual ~FacesClassifier();

	virtual std::list<ROI> *classify();

private:
	CvHaarClassifierCascade *cascade_;
	CvMemStorage *           storage_;
	IplImage *               image_;
	float                    haar_scale_factor_;
	int                      min_neighbours_;
	int                      flags_;
	bool                     own_image_;
};

} // end namespace firevision

#endif
