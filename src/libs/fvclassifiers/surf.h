
/***************************************************************************
 *  feature.h - Feature-based classifier using OpenCV structures
 *
 *  Created: Mon Mar 15 15:47:11 2008
 *  Copyright 2008 Stefan Schiffer [stefanschiffer.de]
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

#ifndef __FIREVISION_CLASSIFIERS_SURF_H_
#define __FIREVISION_CLASSIFIERS_SURF_H_

#ifndef HAVE_SURF
#  error SURF not available, you may not use the SurfClassifier
#endif

#include <vector>

#include <fvclassifiers/classifier.h>

// FIXME replace with forward declarations
#include <surf/ipoint.h>
#include <surf/image.h>
//class surf::Ipoint;
//class surf::Image;
#include <utils/time/clock.h>
#include <utils/time/tracker.h>

//#define NUM_OBJ 13
#define OFFLINE_SURF true  // offline reading - reading from descriptors folder
#define MIN_MATCH_RATIO 0.05

//#ifdef SURF_TIMETRACKER
namespace fawkes {
    class TimeTracker;
}
//#endif

//struct CvMemStorage;
//typedef struct _IplImage IplImage;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void saveIpoints(std::string sFileName, const std::vector< surf::Ipoint >& ipts, bool bVerbose, bool bLaplacian, int VLength);

void loadIpoints(std::string sFileName, std::vector< surf::Ipoint >& ipts, bool bVerbose, int&);

class SurfClassifier : public Classifier
{
 public:
	 //instantiating using descriptor files
  SurfClassifier( std::string keypoints_descriptor_txt_file,
		  unsigned int min_match = 5,
		  float min_match_ratio = MIN_MATCH_RATIO,
		  int samplingStep = 2,
		  int octaves = 4,
		  double thres = 4.0,
		  bool doubleImageSize = false,
		  int initLobe = 3,
		  bool upright = false,
		  bool extended = false,
		  int indexSize = 4);

	//instantiating using a directory containing the png images
  SurfClassifier(const char * image_directory_png_files,
		 unsigned int min_match = 5,
		 float min_match_ratio = MIN_MATCH_RATIO,
		 int samplingStep = 2,
		 int octaves = 4,
		 double thres = 4.0,
		 bool doubleImageSize = false,
		 int initLobe = 3,
		 bool upright = false,
		 bool extended = false,
		 int indexSize = 4);

  virtual ~SurfClassifier();

  virtual std::list< ROI > * classify();

 private:

  unsigned int __num_obj; // number of objects

  // Find closest interest point in a list, given one interest point
  int findMatch(const surf::Ipoint& ip1, const std::vector< surf::Ipoint >& ipts);

  // Calculate square distance of two vectors
  double distSquare(double *v1, double *v2, int n);

  // Object objects
  surf::Image *__obj_img;
  std::vector<std::vector< surf::Ipoint > > __obj_features;
  std::vector<std::string> __obj_names;
  int __obj_num_features;

  // Image objects
  surf::Image *__image;
  std::vector< surf::Ipoint > __img_features;
  int __img_num_features;

  // minimum (absolute) number of features that have to be matched per ROI
  unsigned int __min_match;
  // minimum ratio of features per total object-features that have to be matched per ROI
  float __min_match_ratio;

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
  fawkes::TimeTracker *__tt;
  unsigned int __loop_count;
  unsigned int __ttc_objconv;
  unsigned int __ttc_objfeat;
  unsigned int __ttc_imgconv;
  unsigned int __ttc_imgfeat;
  unsigned int __ttc_matchin;
  unsigned int __ttc_roimerg;
  //#endif

};

} // end namespace firevision

#endif
