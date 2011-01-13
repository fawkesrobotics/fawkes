
/***************************************************************************
 *  sift.cpp - Feature-based classifier using OpenCV structures
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

#include <iostream>
#include <vector>

#include <fvclassifiers/sift.h>
//#ifdef SIFT_TIMETRACKER
#include <utils/time/clock.h>
#include <utils/time/tracker.h>
//#endif

extern "C" {
#include <sift/sift.h>
#include <sift/imgfeatures.h>
#include <sift/kdtree.h>
#include <sift/utils.h>
#include <sift/xform.h>
}

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SiftClassifier <fvclassifiers/sift.h>
 * SIFT classifier.
 *
 * This class provides a classifier that uses OpenCV to detect objects in a given
 * image by matching features using SIFT. The objects are reported back as regions
 * of interest.  Each ROI contains an object.
 *
 * This code is based on the sift package provided by Rob Hess.
 * at http://web.engr.oregonstate.edu/~hess/
 *
 * @author Stefan Schiffer
 */

/** Constructor.
 * @param object_file file that contains the object to detect
 * @param pixel_width width of images that will be processed
 * @param pixel_height height of images that will be processed
 * @param kdtree_bbf_max_nn_chks maximum number of keypoint NN candidates to check during BBF search
 * @param nn_sq_dist_ratio_thr threshold on squared ratio of distances between NN and 2nd NN
 * @param flags flags, not used yet.
 */
SiftClassifier::SiftClassifier( const char * object_file,
				      unsigned int pixel_width, unsigned int pixel_height,
				      int kdtree_bbf_max_nn_chks, float nn_sq_dist_ratio_thr, int flags)
  : Classifier("SiftClassifier")
{
  __kdtree_bbf_max_nn_chks = kdtree_bbf_max_nn_chks;
  __nn_sq_dist_ratio_thr = nn_sq_dist_ratio_thr;
  __flags = flags;


  //#ifdef SIFT_TIMETRACKER
  __tt = new TimeTracker();
  __loop_count = 0;
  __ttc_objconv = __tt->add_class("ObjectConvert");
  __ttc_objfeat = __tt->add_class("ObjectFeatures");
  __ttc_imgconv = __tt->add_class("ImageConvert");
  __ttc_imgfeat = __tt->add_class("ImageFeatures");
  __ttc_matchin = __tt->add_class("Matching");
  __ttc_roimerg = __tt->add_class("MergeROIs");
  //#endif

  //#ifdef SIFT_TIMETRACKER
  __tt->ping_start(__ttc_objconv);
  //#endif
  __obj_img = cvLoadImage( object_file, 1 );
  if ( ! __obj_img ) {
    throw Exception("Could not load object file");
  }
  //#ifdef SIFT_TIMETRACKER
  __tt->ping_end(__ttc_objconv);
  //#endif

  //#ifdef SIFT_TIMETRACKER
  __tt->ping_start(__ttc_objfeat);
  //#endif
  __obj_num_features = 0;
  __obj_num_features = sift_features( __obj_img, &__obj_features );
  if ( ! __obj_num_features > 0 ) {
    throw Exception("Could not compute object features");
  }
  std::cout << "SiftClassifier(classify): computed '" << __obj_num_features << "' features from object" << std::endl;
  //cvReleaseImage(&__obj_img);
  //#ifdef SIFT_TIMETRACKER
  __tt->ping_end(__ttc_objfeat);
  //#endif

  // create space for OpenCV image
  __image = cvCreateImage(cvSize(pixel_width, pixel_height), IPL_DEPTH_8U, 3);

}


/** Destructor. */
SiftClassifier::~SiftClassifier()
{
  //
  cvReleaseImage(&__obj_img);
  cvReleaseImage(&__image);
}


std::list< ROI > *
SiftClassifier::classify()
{
  //#ifdef SIFT_TIMETRACKER
  __tt->ping_start(0);
  //#endif

  // list of ROIs to return
  std::list< ROI > *rv = new std::list< ROI >();

  struct feature * feat;
  struct feature** nbrs;
  struct kd_node* kd_root;
  CvPoint pt1, pt2;

  // for ROI calculation
  CvPoint ftpt;
  std::vector< CvPoint > ftlist;
  //= new std::vector< CvPoint >();
  int x_min = _width;
  int y_min = _height;
  int x_max = 0;
  int y_max = 0;

  double d0, d1;// = 0.0;
  int k, m = 0;

  //#ifdef SIFT_TIMETRACKER
  __tt->ping_start(__ttc_imgconv);
  //#endif
  //std::cout << "SiftClassifier(classify): convert frame to IplImage" << std::endl;
  convert(YUV422_PLANAR, BGR, _src, (unsigned char *)__image->imageData, _width, _height);
  //#ifdef SIFT_TIMETRACKER
  __tt->ping_end(__ttc_imgconv);
  //#endif

  //#ifdef SIFT_TIMETRACKER
  __tt->ping_start(__ttc_imgfeat);
  //#endif
  //std::cout << "SiftClassifier(classify): compute features on current frame " << std::endl;
  int num_img_ft = sift_features( __image, &__img_features );
  kd_root = kdtree_build( __img_features, num_img_ft );
  //#ifdef SIFT_TIMETRACKER
  __tt->ping_end(__ttc_imgfeat);
  //#endif

  if( ! kd_root ) {
    std::cerr << "SiftClassifier(classify): KD-Root NULL!" << std::endl;
  }

  //#ifdef SIFT_TIMETRACKER
  __tt->ping_start(__ttc_matchin);
  //#endif
  std::cout << "SiftClassifier(classify): matching ..." << std::endl;
  for( int i = 0; i < __obj_num_features; ++i ) {
    //std::cout << "SiftClassifier(classify): ... feature '" << i << "'" << std::endl;
    feat = __obj_features + i;
    k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, __kdtree_bbf_max_nn_chks );
    if( k == 2 )
      {
 	d0 = descr_dist_sq( feat, nbrs[0] );
 	d1 = descr_dist_sq( feat, nbrs[1] );
 	if( d0 < d1 * __nn_sq_dist_ratio_thr )
 	  {
 	    pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
 	    pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
 	    m++;
 	    __obj_features[i].fwd_match = nbrs[0];
	    // save matched feature points
 	    ftpt = cvPoint( cvRound( nbrs[0]->x), cvRound( nbrs[0]->y ) );
	    ftlist.push_back(ftpt);
	    // save matched features as ROIs
     	    ROI r( pt2.x-5, pt2.y-5, 11, 11, _width, _height);
     	    rv->push_back(r);
 	  }
       }
     free( nbrs );
  }
  std::cout << "SiftClassifier(classify): found '" << m << "' matches" << std::endl;
  kdtree_release( kd_root );
  //#ifdef SIFT_TIMETRACKER
  __tt->ping_end(__ttc_matchin);
  //#endif

  //#ifdef SIFT_TIMETRACKER
  __tt->ping_start(__ttc_roimerg);
  //#endif
  std::cout << "SiftClassifier(classify): computing ROI" << std::endl;
  //for ( int i = 0; i < m; ++i) {
  for ( std::vector< CvPoint >::size_type i = 0; i < ftlist.size(); ++i) {
    if( ftlist[i].x < x_min )
      x_min = ftlist[i].x;
    if( ftlist[i].y < y_min )
      y_min = ftlist[i].y;
    if( ftlist[i].x > x_max )
      x_max = ftlist[i].x;
    if( ftlist[i].y > y_max )
      y_max = ftlist[i].y;
  }
  if( m != 0 ) {
    ROI r(x_min, y_min, x_max-x_min, y_max-y_min, _width, _height);
    rv->push_back(r);
  }
  //#ifdef SIFT_TIMETRACKER
  __tt->ping_end(__ttc_roimerg);
  //#endif

  //#ifdef SIFT_TIMETRACKER
  __tt->ping_end(0);
  //#endif

  //#ifdef SIFT_TIMETRACKER
  __tt->print_to_stdout();
  //#endif

  std::cout << "SiftClassifier(classify): done ... returning '" << rv->size() << "' ROIs." << std::endl;
  return rv;
}

} // end namespace firevision
