
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

#ifndef _FIREVISION_CLASSIFIERS_SIFTPP_H_
#define _FIREVISION_CLASSIFIERS_SIFTPP_H_

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
  VL::PgmBuffer      *obj_img_;
  std::vector< Feature > obj_features_;
  int obj_num_features_;

  // Image objects
  VL::PgmBuffer      *image_;
  std::vector< Feature > img_features_;
  int img_num_features_;

  // Initial sampling step (default 2)
  int samplingStep_;
  // Number of analysed octaves (default 4)
  int octaves_;
  // Number of levels per octave (default 3)
  int levels_;
  // Blob response treshold
  VL::float_t threshold_;
  VL::float_t edgeThreshold_;

  int   first_;

  //  float const sigman_;
  //  float const sigma0_;
  float sigman_;
  float sigma0_;

  // Keypoint magnification (default 3)
  float magnif_;
  // Upright SIFTPP or rotation invaraiant
  int   noorient_;
  // Normalize decriptors?
  int   unnormalized_;

  // UNUSED
//   int    stableorder    = 0 ;
//   int    savegss        = 0 ;
//   int    verbose        = 0 ;
//   int    binary         = 0 ;
//   int    haveKeypoints  = 0 ;
//   int    fp             = 0 ;

  // Length of descriptor vector
  int vlen_;

  //#ifdef SIFTPP_TIMETRACKER
  fawkes::TimeTracker *tt_;
  unsigned int loop_count_;
  unsigned int ttc_objconv_;
  unsigned int ttc_objfeat_;
  unsigned int ttc_imgconv_;
  unsigned int ttc_imgfeat_;
  unsigned int ttc_matchin_;
  unsigned int ttc_roimerg_;
  //#endif

};

} // end namespace firevision

#endif
