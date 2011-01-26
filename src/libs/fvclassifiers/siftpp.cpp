
/***************************************************************************
 *  siftpp.cpp - siftpp based classifier 
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

#include <iostream>
#include <vector>

#include <fvclassifiers/siftpp.h>

//#ifdef SIFTPP_TIMETRACKER
#include <utils/time/clock.h>
#include <utils/time/tracker.h>
//#endif

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <fvutils/readers/png.h>
//#include <fvutils/writers/pnm.h>
//#include <fvutils/writers/png.h>

//using namespace fawkes; 
using namespace fawkes; 

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SiftppClassifier <fvclassifiers/siftpp.h>
 * SIFTPP classifier.
 *
 * This class provides a classifier that uses SIFTPP to detect objects in a given
 * image by matching features. The objects are reported back as regions of interest. 
 * Each ROI contains an object. ROIs with 11x11 are matched features.
 *
 * This code uses siftpp from http://vision.ucla.edu/~vedaldi/code/siftpp/siftpp.html
 * and is partly based on code from their package.
 *
 * @author Stefan Schiffer
 */

/** Constructor.
 * @param object_file file that contains an image of the object to detect
 * @param samplingStep Initial sampling step
 * @param octaves Number of analysed octaves
 * @param levels Number of levels per octave
 * @param magnif Keypoint magnification (default = 3)
 * @param noorient rotation invariance (0) or upright (1)
 * @param unnormalized Normalization of features (default 0)
 */
SiftppClassifier::SiftppClassifier( const char * object_file,
				    int samplingStep, int octaves, int levels,
				    float magnif, int noorient, int unnormalized)
  : Classifier("SiftppClassifier")
{
  // params for FastHessian
  __samplingStep = samplingStep;
  __octaves = octaves;
  __levels = levels;
  // params for Descriptors
  __first          = -1 ;
  __threshold      = 0.04f / __levels / 2.0f ;
  __edgeThreshold  = 10.0f;
  __magnif         = magnif;
  __noorient       = noorient;
  __unnormalized   = unnormalized;

  // descriptor vector length
  __vlen = 128;


  //#ifdef SIFTPP_TIMETRACKER
  __tt = new TimeTracker();
  __loop_count = 0;
  __ttc_objconv = __tt->add_class("ObjectConvert");
  __ttc_objfeat = __tt->add_class("ObjectFeatures");
  __ttc_imgconv = __tt->add_class("ImageConvert");
  __ttc_imgfeat = __tt->add_class("ImageFeatures");
  __ttc_matchin = __tt->add_class("Matching");
  __ttc_roimerg = __tt->add_class("MergeROIs");
  //#endif

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_start(__ttc_objconv);
  //#endif
  
  PNGReader pngr( object_file );
  unsigned char* buf = malloc_buffer( pngr.colorspace(), pngr.pixel_width(), pngr.pixel_height() );
  pngr.set_buffer( buf );
  pngr.read();
  
  unsigned int lwidth = pngr.pixel_width();
  unsigned int lheight = pngr.pixel_height();
  VL::pixel_t * im_pt = new VL::pixel_t [lwidth * lheight ];
  VL::pixel_t * start = im_pt;
  //VL::pixel_t* end   = start + lwidth*lheight ; 
  for (unsigned int h = 0; h < lheight; ++h) {
    for (unsigned int w = 0; w < lwidth ; ++w) {
      int i = (buf[h * lwidth + w] );
      VL::pixel_t norm = VL::pixel_t( 255 );
      *start++ = VL::pixel_t( i ) / norm;
    }
  }
  // make image
  __obj_img = new VL::PgmBuffer();
  __obj_img->width  = lwidth;
  __obj_img->height = lheight;
  __obj_img->data   = im_pt;

  if ( ! __obj_img ) {
    throw Exception("Could not load object file");
  }

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_end(__ttc_objconv);
  //#endif

  // save object image for debugging
  //

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_start(__ttc_objfeat);
  //#endif

  // COMPUTE OBJECT FEATURES
  __obj_features.clear();
  //__obj_features.reserve(1000);
  __obj_num_features = 0;

  __sigman = .5 ;
  __sigma0 = 1.6 * powf(2.0f, 1.0f / __levels) ;

  std::cout << "SiftppClassifier(ctor): init scalespace" << std::endl;
  // initialize scalespace
  VL::Sift sift(__obj_img->data, __obj_img->width, __obj_img->height, 
		__sigman, __sigma0, __octaves, __levels, __first, -1, __levels+1) ;
  
  std::cout << "SiftppClassifier(ctor): detect object keypoints" << std::endl;
  // Run SIFTPP detector
  sift.detectKeypoints(__threshold, __edgeThreshold) ;
  // Number of keypoints
  __obj_num_features = sift.keypointsEnd() - sift.keypointsBegin();
  std::cout << "SiftppClassifier(ctor): computed '" << __obj_num_features << "' object-keypoints" << std::endl;

  // set descriptor options
  sift.setNormalizeDescriptor( ! __unnormalized ) ;
  sift.setMagnification( __magnif ) ;

  std::cout << "SiftppClassifier(ctor): run detector, compute ori and des ..." << std::endl;
  // Run detector, compute orientations and descriptors
  for( VL::Sift::KeypointsConstIter iter = sift.keypointsBegin() ;
       iter != sift.keypointsEnd() ; ++iter ) {

    //Feature * feat = new Feature();
    Feature feat;

    //std::cout << "SiftppClassifier(ctor): saving keypoint" << std::endl;
    feat.key = (*iter);

    // detect orientations
    VL::float_t angles [4] ;
    int nangles ;
    if( ! __noorient ) {
      nangles = sift.computeKeypointOrientations(angles, *iter) ;
    } else {
      nangles = 1;
      angles[0] = VL::float_t(0) ;
    }
    feat.number_of_desc = nangles;
    feat.descs = new VL::float_t*[nangles];
    
    //std::cout << "SiftppClassifier(ctor): computing '" << nangles << "' descriptors" << std::endl;
    // compute descriptors
    for(int a = 0 ; a < nangles ; ++a) {
      //       out << setprecision(2) << iter->x << ' ' << setprecision(2) << iter->y << ' '
      // 	  << setprecision(2) << iter->sigma << ' ' << setprecision(3) << angles[a] ;
      // compute descriptor
      feat.descs[a] = new VL::float_t[__vlen];
      sift.computeKeypointDescriptor(feat.descs[a], *iter, angles[a]) ;
    } // next angle
    //std::cout << "SiftppClassifier(ctor): computed '" << feat.number_of_desc << "' descriptors." << std::endl;

    // save feature
    __obj_features.push_back( feat );

  } // next keypoint
  
  __obj_num_features = __obj_features.size();
  if ( ! __obj_num_features > 0 ) {
    throw Exception("Could not compute object features");
  }
  std::cout << "SiftppClassifier(ctor): computed '" << __obj_num_features << "' features from object" << std::endl;

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_end(__ttc_objfeat);
  //#endif

}


/** Destructor. */
SiftppClassifier::~SiftppClassifier()
{
  //
  delete __obj_img;
  __obj_features.clear();
  //
  //delete __image;
  __img_features.clear();
}


std::list< ROI > *
SiftppClassifier::classify()
{
  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_start(0);
  //#endif

  // list of ROIs to return
  std::list< ROI > *rv = new std::list< ROI >();

  // for ROI calculation
  int x_min = _width;
  int y_min = _height;
  int x_max = 0;
  int y_max = 0;
  
  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_start(__ttc_imgconv);
  //#endif
  std::cout << "SiftppClassifier(classify): copy imgdat to SIFTPP Image" << std::endl;

  VL::pixel_t * im_pt = new VL::pixel_t [_width * _height ];
  VL::pixel_t * start = im_pt;
  for (unsigned int h = 0; h < _height; ++h) {
    for (unsigned int w = 0; w < _width ; ++w) {
      int i = (_src[h * _width + w] );
      VL::pixel_t norm = VL::pixel_t( 255 );
      *start++ = VL::pixel_t( i ) / norm;
    }
  }
  // make image
  __image = new VL::PgmBuffer();
  __image->width  = _width;
  __image->height = _height;
  __image->data   = im_pt;

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_end(__ttc_imgconv);
  //#endif

  /// Write image to verify correct operation
    // nothing yet

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_start(__ttc_imgfeat);
  //#endif

  // COMPUTE IMAGE FEATURES
  __img_features.clear();
  __img_num_features = 0;
  //__img_features.reserve(1000);

  std::cout << "SiftppClassifier(classify): init scalespace" << std::endl;
  // initialize scalespace
  VL::Sift sift(__image->data, __image->width, __image->height, 
		__sigman, __sigma0, __octaves, __levels, __first, -1, __levels+1) ;
  
  std::cout << "SiftppClassifier(classify): detect image keypoints" << std::endl;
  // Run SIFTPP detector
  sift.detectKeypoints(__threshold, __edgeThreshold) ;

  // Number of keypoints
  __img_num_features = sift.keypointsEnd() - sift.keypointsBegin();
  std::cout << "SiftppClassifier(classify): Extracted '" << __img_num_features << "' image keypoints" << std::endl;

  // set descriptor options
  sift.setNormalizeDescriptor( ! __unnormalized ) ;
  sift.setMagnification( __magnif ) ;

  std::cout << "SiftppClassifier(classify): run detector, compute ori and des ..." << std::endl;
  // Run detector, compute orientations and descriptors
  for( VL::Sift::KeypointsConstIter iter = sift.keypointsBegin() ;
       iter != sift.keypointsEnd() ; ++iter ) {

    Feature feat; // = new Feature();
    
    //std::cout << "SiftppClassifier(classify): saving keypoint" << std::endl;
    feat.key = (*iter);

    //std::cout << "SiftppClassifier(classify): detect orientations" << std::endl;
    // detect orientations
    VL::float_t angles [4] ;
    int nangles ;
    if( ! __noorient ) {
      nangles = sift.computeKeypointOrientations(angles, *iter) ;
    } else {
      nangles = 1;
      angles[0] = VL::float_t(0) ;
    }
    feat.number_of_desc = nangles;
    feat.descs = new VL::float_t*[nangles];
    
    //std::cout << "SiftppClassifier(classify): computing '" << nangles << "' descriptors" << std::endl;
    // compute descriptors
    for(int a = 0 ; a < nangles ; ++a) {
      // compute descriptor
      feat.descs[a] = new VL::float_t[__vlen] ;
      sift.computeKeypointDescriptor(feat.descs[a], *iter, angles[a]) ;
    } // next angle
    //std::cout << "SiftppClassifier(classify): computed '" << feat.number_of_desc << "' descriptors." << std::endl;

    // save feature
    __img_features.push_back( feat );

  } // next keypoint

  // Number of feature
  __img_num_features = __img_features.size();

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_end(__ttc_imgfeat);
  //#endif

  std::cout << "SiftppClassifier(classify): Extracted '" << __img_num_features << "' image features" << std::endl;

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_start(__ttc_matchin);
  //#endif
  std::cout << "SiftppClassifier(classify): matching ..." << std::endl;

  std::vector< int > matches(__obj_features.size());
  int m = 0;
  for (unsigned i = 0; i < __obj_features.size(); i++) {
    int match = findMatch(__obj_features[i], __img_features);
    matches[i] = match;
    if (match != -1) {
      std::cout << "SiftppClassifier(classify): Matched feature " << i << " in object image with feature " << match << " in image." << std::endl;
      /// adding feature-ROI
      ROI r( (int)(__img_features[matches[i]].key.x)-5, (int)(__img_features[matches[i]].key.y )-5, 11, 11, _width, _height);
      rv->push_back(r);
      // increment feature-match-count
      ++m;
    }
  }

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_end(__ttc_matchin);
  //#endif
  std::cout << "SiftppClassifier(classify) matched '" << m << "' of '" << __obj_features.size() << "' features in scene." << std::endl;

  std::cout << "SiftppClassifier(classify): computing ROI" << std::endl;
  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_start(__ttc_roimerg);
  //#endif
  
  for (unsigned i = 0; i < matches.size(); i++) {
    if (matches[i] != -1) {
      if( (int)__img_features[matches[i]].key.x < x_min )
 	x_min = (int)__img_features[matches[i]].key.x;
      if( (int)__img_features[matches[i]].key.y < y_min )
 	y_min = (int)__img_features[matches[i]].key.y;
      if( (int)__img_features[matches[i]].key.x > x_max )
 	x_max = (int)__img_features[matches[i]].key.x;
      if( (int)__img_features[matches[i]].key.y > y_max )
 	y_max = (int)__img_features[matches[i]].key.y;
    }
  }
  if( m != 0 ) {
    ROI r(x_min, y_min, x_max-x_min, y_max-y_min, _width, _height);
    rv->push_back(r);
  }
  
  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_end(__ttc_roimerg);
  //#endif

  //#ifdef SIFTPP_TIMETRACKER
  __tt->ping_end(0);
  //#endif

  //#ifdef SIFTPP_TIMETRACKER
  // print timetracker statistics
  __tt->print_to_stdout();
  //#endif

  delete __image;

  std::cout << "SiftppClassifier(classify): done ... returning '" << rv->size() << "' ROIs." << std::endl;
  return rv;
}

int
SiftppClassifier::findMatch(const Feature & ip1, const std::vector< Feature > & ipts) {
  double mind = 1e100, second = 1e100;
  int match = -1;
  
  for (unsigned i = 0; i < ipts.size(); i++) {

    if (ipts[i].number_of_desc != ip1.number_of_desc)
      continue;
    //std::cout << "SiftppClassifier(findMatch): number_of_desc matched!" << std::endl;
    for ( int j = 0; j < ip1.number_of_desc; ++j ) {
      double d = distSquare(ipts[i].descs[j], ip1.descs[j], __vlen);
      
      if (d < mind) {
	second = mind;
	mind = d;
	match = i;
      } else if (d < second) {
	second = d;
      }
    }
  }
  
  if (mind < 0.5 * second)
    return match;
  
  return -1;
}


double
SiftppClassifier::distSquare(VL::float_t *v1, VL::float_t *v2, int n) {
  double dsq = 0.;
  while (n--) {
    dsq += (v1[n-1] - v2[n-1]) * (v1[n-1] - v2[n-1]);
  }
  //std::cout << "  dsq: '" << dsq << "'" << std::endl;
  return dsq;
}

} // end namespace firevision
