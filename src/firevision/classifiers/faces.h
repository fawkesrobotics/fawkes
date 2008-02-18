
/***************************************************************************
 *  faces.h - Faces classifier based on OpenCV
 *
 *  Created: Mon Dec 10 15:46:06 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CLASSIFIERS_FACES_H_
#define __FIREVISION_CLASSIFIERS_FACES_H_

#include <classifiers/classifier.h>

struct CvHaarClassifierCascade;
struct CvMemStorage;
typedef struct _IplImage IplImage;

class FacesClassifier : public Classifier
{
 public:
  FacesClassifier(const char *haarcascade_file,
		  unsigned int pixel_width, unsigned int pixel_height,
		  float haar_scale_factor = 1.1, int min_neighbours = 3, int flags = 0);

  virtual ~FacesClassifier(); 

  virtual std::list< ROI > * classify();

 private:
  CvHaarClassifierCascade *__cascade;
  CvMemStorage *__storage;
  IplImage *__image;
  float __haar_scale_factor;
  int __min_neighbours;
  int __flags;
};

#endif
