
/***************************************************************************
 *  sift.h - Feature-based classifier using OpenCV structures
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

#ifndef __FIREVISION_CLASSIFIERS_SIFT_H_
#define __FIREVISION_CLASSIFIERS_SIFT_H_

#ifndef HAVE_SIFT
#  error SIFT not available, you may not use the SiftClassifier
#endif

#include <fvclassifiers/classifier.h>

//#ifdef FEAT_TIMETRACKER
namespace fawkes {
  class TimeTracker;
}
//#endif

struct CvMemStorage;
typedef struct _IplImage IplImage;

struct feature;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif
class SiftClassifier : public Classifier
{
 public:
  SiftClassifier(const char * features_file,
		    unsigned int pixel_width, unsigned int pixel_height,
		    int kdtree_bbf_max_nn_chks = 200, float nn_sq_dist_ratio_thr = 0.49, int flags = 0);
  
  virtual ~SiftClassifier(); 
  
  virtual std::list< ROI > * classify();
  
 private:
  
  const char ** __features_files;

  IplImage *__obj_img;
  feature *__obj_features;
  int __obj_num_features;

  CvMemStorage *__storage;
  IplImage *__image;

  feature* __img_features;
  
  float __nn_sq_dist_ratio_thr;
  int __kdtree_bbf_max_nn_chks;
  int __flags;

  //#ifdef FEAT_TIMETRACKER
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
