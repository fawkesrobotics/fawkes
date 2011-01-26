
/***************************************************************************
 *  siftpp.h - Feature-based classifier using siftpp
 *
 *  Created: Sat Apr 12 10:15:23 2008
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

#ifndef __FIREVISION_CLASSIFIERS_SIFTPP_H_
#define __FIREVISION_CLASSIFIERS_SIFTPP_H_

#ifndef HAVE_SIFTPP
#  error SIFTPP not available, you may not use the SiftppClassifier
#endif

#include <vector>
#include <utils/time/clock.h>
#include <utils/time/tracker.h>

#include <fvclassifiers/classifier.h>

// FIXME replace with forward declarations
#include <siftpp/sift.hpp>

//#ifdef SIFTPP_TIMETRACKER

class fawkes::TimeTracker;
//#endif

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SiftppClassifier : public Classifier
{
 public:
  SiftppClassifier(const char * features_file,
		   int samplingStep = 2,
		   int octaves = 4,
		   int levels = 3,
		   float magnif = 3.0,
		   int noorient = 0,
		   int unnormalized = 0);
  
  virtual ~SiftppClassifier(); 
  
  virtual std::list< ROI > * classify();

  /** Siftpp Feature struct. */
  struct Feature {
    VL::Sift::Keypoint key;    /**< keypoint */
    int     number_of_desc;    /**< number of descriptors */
    VL::float_t ** descs;      /**< descriptors */
  };

 private:
  
  // Find closest interest point in a list, given one interest point
  int findMatch(const Feature & ip1, const std::vector< Feature > & ipts);

  // Calculate square distance of two vectors
  //double distSquare(double *v1, double *v2, int n);
  double distSquare(VL::float_t *v1, VL::float_t *v2, int n);

  // Object objects
  VL::PgmBuffer      *__obj_img;
  std::vector< Feature > __obj_features;
  int __obj_num_features;

  // Image objects
  VL::PgmBuffer      *__image;
  std::vector< Feature > __img_features;
  int __img_num_features;

  // Initial sampling step (default 2)
  int __samplingStep;
  // Number of analysed octaves (default 4)
  int __octaves;
  // Number of levels per octave (default 3)
  int __levels;
  // Blob response treshold
  VL::float_t __threshold;
  VL::float_t __edgeThreshold;

  int   __first;

  //  float const __sigman;
  //  float const __sigma0;
  float __sigman;
  float __sigma0;

  // Keypoint magnification (default 3)
  float __magnif;
  // Upright SIFTPP or rotation invaraiant
  int   __noorient;
  // Normalize decriptors?
  int   __unnormalized;

  // UNUSED
//   int    stableorder    = 0 ;
//   int    savegss        = 0 ;
//   int    verbose        = 0 ;
//   int    binary         = 0 ;
//   int    haveKeypoints  = 0 ;
//   int    fp             = 0 ;

  // Length of descriptor vector
  int __vlen;

  //#ifdef SIFTPP_TIMETRACKER
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
