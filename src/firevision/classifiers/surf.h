
/***************************************************************************
 *  feature.h - Feature-based classifier using OpenCV structures
 *
 *  Created: Mon Mar 15 15:47:11 2008
 *  Copyright 2008 Stefan Schiffer [stefanschiffer.de]
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

#ifndef __FIREVISION_CLASSIFIERS_SURF_H_
#define __FIREVISION_CLASSIFIERS_SURF_H_

#ifndef HAVE_SURF
#  error SURF not available, you may not use the SurfClassifier
#endif

#include <vector>

#include <classifiers/classifier.h>

// FIXME replace with forward declarations
#include <surf/ipoint.h>
#include <surf/image.h>
//class surf::Ipoint;
//class surf::Image;

//#ifdef SURF_TIMETRACKER
class TimeTracker;
//#endif

//struct CvMemStorage;
//typedef struct _IplImage IplImage;

class SurfClassifier : public Classifier
{
 public:
  SurfClassifier(const char * features_file,
		 unsigned int pixel_width, unsigned int pixel_height,
		 // Initial sampling step (default 2)
		 int samplingStep = 2,
		 // Number of analysed octaves (default 4)
		 int octaves = 4,
		 // Blob response treshold
		 double thres = 4.0,
		 // Set this flag "true" to double the image size
		 bool doubleImageSize = false,
		 // Initial lobe size, default 3 and 5 (with double image size)
		 int initLobe = 3,
		 // Upright SURF or rotation invaraiant
		 bool upright = false,
		 // If the extended flag is turned on, SURF 128 is used
		 bool extended = false,
		 // Spatial size of the descriptor window (default 4)
		 int indexSize = 4,
		 // flags
		 int flags = 0);
  
  virtual ~SurfClassifier(); 
  
  virtual std::list< ROI > * classify();
  
 private:
  
  // Find closest interest point in a list, given one interest point
  int findMatch(const surf::Ipoint& ip1, const std::vector< surf::Ipoint >& ipts);

  // Calculate square distance of two vectors
  double distSquare(double *v1, double *v2, int n);

  // Object objects
  surf::Image *__obj_img;
  std::vector< surf::Ipoint > __obj_features;
  int __obj_num_features;

  // Image objects
  surf::Image *__image;
  std::vector< surf::Ipoint > __img_features;
  int __img_num_features;

  // c'tor parameters
  int __flags;

  // Initial sampling step (default 2)
  int __samplingStep;
  // Number of analysed octaves (default 4)
  int __octaves;
  // Blob response treshold
  double __thres;
  // Set this flag "true" to double the image size
  bool __doubleImageSize;
  // Initial lobe size, default 3 and 5 (with double image size)
  int __initLobe;
  // Upright SURF or rotation invaraiant
  bool __upright;
  // If the extended flag is turned on, SURF 128 is used
  bool __extended;
  // Spatial size of the descriptor window (default 4)
  int __indexSize;

  // Length of descriptor vector
  int __vlen;

  //#ifdef SURF_TIMETRACKER
  TimeTracker *__tt;
  unsigned int __loop_count;
  unsigned int __ttc_objconv;
  unsigned int __ttc_objfeat;
  unsigned int __ttc_imgconv;
  unsigned int __ttc_imgfeat;
  unsigned int __ttc_matchin;
  unsigned int __ttc_roimerg;
  //#endif

};

#endif
