
/***************************************************************************
 *  surf.cpp - SURF based classifier 
 *
 *  Created: Tue Apr 01 10:15:23 2008
 *  Copyright 2008 Stefan Schiffer [stefanschiffer.de]
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

#include <iostream>
#include <vector>

#include <classifiers/surf.h>

//#ifdef SURF_TIMETRACKER
#include <utils/time/clock.h>
#include <utils/time/tracker.h>
//#endif

#include <surf/surflib.h>
//#include <surf/ipoint.h>
//#include <surf/image.h>
//#include <surf/imload.h>

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <fvutils/readers/png.h>
//#include <fvutils/writers/pnm.h>
//#include <fvutils/writers/png.h>

/** @class SurfClassifier <classifiers/surf.h>
 * SURF classifier.
 *
 * This class provides a classifier that uses SURF to detect objects in a given
 * image by matching features. The objects are reported back as regions of interest. 
 * Each ROI contains an object. ROIs with 11x11 are matched features.
 *
 * This code uses libSurf from http://www.vision.ee.ethz.ch/~surf/
 * and is partly based on code from their package.
 *
 * @author Stefan Schiffer
 */

/** Constructor.
 * @param object_file file that contains an image of the object to detect
 * @param pixel_width width of images that will be processed
 * @param pixel_height height of images that will be processed
 * @param samplingStep Initial sampling step
 * @param octaves Number of analysed octaves
 * @param thres Blob response treshold
 * @param doubleImageSize true to double the image size, false to keep original
 * @param initLobe Initial lobe size, default 3 and 5 (with double image size)
 * @param upright rotation invariance (fasle) or upright (true)
 * @param extended true to use the extended descriptor (SURF 128)
 * @param indexSize Spatial size of the descriptor window (default 4)
 */
SurfClassifier::SurfClassifier( const char * object_file,
				unsigned int pixel_width, unsigned int pixel_height,
				int samplingStep, int octaves, double thres, 
				bool doubleImageSize, int initLobe, 
				bool upright, bool extended, int indexSize)
  : Classifier("SurfClassifier")
{
  // params for FastHessian
  __samplingStep = samplingStep;
  __octaves = octaves;
  __thres = thres;
  __doubleImageSize = doubleImageSize;
  __initLobe = initLobe;
  // params for Descriptors
  __upright = upright;
  __extended = extended;
  __indexSize = indexSize;

  // descriptor vector length
  __vlen = 0;


  //#ifdef SURF_TIMETRACKER
  __tt = new TimeTracker();
  __loop_count = 0;
  __ttc_objconv = __tt->add_class("ObjectConvert");
  __ttc_objfeat = __tt->add_class("ObjectFeatures");
  __ttc_imgconv = __tt->add_class("ImageConvert");
  __ttc_imgfeat = __tt->add_class("ImageFeatures");
  __ttc_matchin = __tt->add_class("Matching");
  __ttc_roimerg = __tt->add_class("MergeROIs");
  //#endif

  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(__ttc_objconv);
  //#endif
  
  PNGReader pngr( object_file );
  unsigned char* buf = malloc_buffer( pngr.colorspace(), pngr.pixel_width(), pngr.pixel_height() );
  pngr.set_buffer( buf );
  pngr.read();
  
  unsigned int lwidth = pngr.pixel_width();
  unsigned int lheight = pngr.pixel_height();
  surf::Image * __simage = new surf::Image( lwidth, lheight );
  for (unsigned int h = 0; h < lheight; ++h) {
    for (unsigned int w = 0; w < lwidth ; ++w) {
      __simage->setPix(w, h, (double)buf[h * lwidth + w] / 255.f);
    }
  }
  // make integral image
  __obj_img = new surf::Image(__simage, __doubleImageSize);

  // NOT WORKING 
  //__obj_img = new surf::Image( pngr.pixel_width(), pngr.pixel_height());
  //__obj_img->setFrame( buf );

  if ( ! __obj_img ) {
    throw Exception("Could not load object file");
  }

  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_objconv);
  //#endif

  // save object image for debugging
  ///surf::ImLoad::saveImage( "obj.pgm", __obj_img);

  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(__ttc_objfeat);
  //#endif

  // COMPUTE OBJECT FEATURES
  __obj_features.clear();
  __obj_features.reserve(1000);
  __obj_num_features = 0;
  // Extract interest points with Fast-Hessian
  surf::FastHessian fh(__obj_img, /* pointer to integral image */
		       __obj_features,
		       __thres, /* blob response threshold */
		       __doubleImageSize, /* double image size flag */
		       __initLobe * 3 /* 3 times lobe size equals the mask size */, 
		       __samplingStep, /* subsample the blob response map */
		       __octaves /* number of octaves to be analysed */);
  // Extract them and get their pointer
  fh.getInterestPoints();
  // Initialise the SURF descriptor
  surf::Surf des(__obj_img, /* pointer to integral image */  
		 __doubleImageSize, /* double image size flag */ 
		 __upright, /* rotation invariance or upright */
		 __extended, /* use the extended descriptor */
		 __indexSize /* square size of the descriptor window (default 4x4)*/);
  // Get the length of the descriptor vector
  // resulting from the parameters
  __vlen = des.getVectLength();

  //printf("vlen=%i\n", __vlen);

  // Compute the orientation and the descriptor for every interest point
  for (unsigned n=0; n < __obj_features.size(); n++){
    // set the current interest point
    des.setIpoint(&__obj_features[n]);
    // assign reproducible orientation
    des.assignOrientation();
    // make the SURF descriptor
    des.makeDescriptor();
  }

  __obj_num_features = __obj_features.size();
  if ( ! __obj_num_features > 0 ) {
    throw Exception("Could not compute object features");
  }
  std::cout << "SurfClassifier(classify): computed '" << __obj_num_features << "' features from object" << std::endl;

  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_objfeat);
  //#endif

}


/** Destructor. */
SurfClassifier::~SurfClassifier()
{
  //
}


std::list< ROI > *
SurfClassifier::classify()
{
  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(0);
  //#endif

  // list of ROIs to return
  std::list< ROI > *rv = new std::list< ROI >();

  // for ROI calculation
  int x_min = _width;
  int y_min = _height;
  int x_max = 0;
  int y_max = 0;

  double d0, d1;// = 0.0;
  int k, i, m = 0;

  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(__ttc_imgconv);
  //#endif
  std::cout << "SurfClassifier(classify): copy imgdat to SURF Image" << std::endl;

  /*
    // NOT WOKRING ALTERNATIVE
  double *tmpb = (double *)malloc(_width * _height * sizeof(double));
  for (unsigned int h = 0; h < _height; ++h) {
    for (unsigned int w = 0; w < _width; ++w) {
      tmpb[h * _width + w] = (double)_src[h * _width + w] / 255;
    }
  }
  __simage->setFrame( (unsigned char*)tmpb );
  //surf::ImLoad::saveImage( "stst.pgm", __simage);
  __image = new surf::Image(__simage, __doubleImageSize);
  //__image = new surf::Image( _width, _height);
  //__image->setFrame( (unsigned char *)tmpb );
  */

  surf::Image * __simage = new surf::Image( _width, _height);
  for (unsigned int h = 0; h < _height; ++h) {
    for (unsigned int w = 0; w < _width; ++w) {
      __simage->setPix(w, h, (double)_src[h * _width + w] / 255.f);
    }
  }
  // create integral image
  __image = new surf::Image(__simage, __doubleImageSize);

  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_imgconv);
  //#endif


  /*
    /// write pnm (with surf-routine) for debugging
    //surf::ImLoad::saveImage( "tst.pgm", __simage);
    /// write integral pnm (with surf-routine) for debugging
    //surf::ImLoad::saveImage( "tst.pgm", __image);
    /// write pgm (with fv-routine) for debugging
    PNMWriter pnm(PNM_PGM, "fvimg.pgm", _width, _height);
    pnm.set_buffer(YUV422_PLANAR, _src );
    pnm.write();
    /// write png (with fv-routine) for debugging
    PNGWriter pngw("fvimg.png", _width, _height);
    pngw.set_buffer(YUV422_PLANAR, _src );
    pngw.write();
  */

  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(__ttc_imgfeat);
  //#endif

  // COMPUTE OBJECT FEATURES
  __img_features.clear();
  __img_features.reserve(1000);
  __img_num_features = 0;
  // Extract interest points with Fast-Hessian
  surf::FastHessian fh(__image, /* pointer to integral image */
		       __img_features,
		       __thres, /* blob response threshold */
		       __doubleImageSize, /* double image size flag */
		       __initLobe * 3 /* 3 times lobe size equals the mask size */, 
		       __samplingStep, /* subsample the blob response map */
		       __octaves /* number of octaves to be analysed */);
  // Extract them and get their pointer
  fh.getInterestPoints();
  // Initialise the SURF descriptor
  surf::Surf des(__image, /* pointer to integral image */  
		 __doubleImageSize, /* double image size flag */ 
		 __upright, /* rotation invariance or upright */
		 __extended, /* use the extended descriptor */
		 __indexSize /* square size of the descriptor window (default 4x4)*/);
  // Get the length of the descriptor vector
  // resulting from the parameters
  // NOT NEEDED HERE!
  //__vlen = des.getVectLength();
  //printf("img vlen=%i\n", __vlen);

  // Compute the orientation and the descriptor for every interest point
  for (unsigned n=0; n < __img_features.size(); n++){
    //for (Ipoint *k = ipts; k != NULL; k = k->next){
    // set the current interest point
    des.setIpoint(&__img_features[n]);
    // assign reproducible orientation
    des.assignOrientation();
    // make the SURF descriptor
    des.makeDescriptor();
  }
  __img_num_features = __img_features.size();
  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_imgfeat);
  //#endif

  std::cout << "Extracted '" << __img_num_features << "' image features" << std::endl;


  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(__ttc_matchin);
  //#endif
  std::cout << "SurfClassifier(classify): matching ..." << std::endl;

  std::vector< int > matches(__obj_features.size());
  int c = 0;
  for (unsigned i = 0; i < __obj_features.size(); i++) {
    int match = findMatch(__obj_features[i], __img_features);
    matches[i] = match;
    if (match != -1) {
      // std::cout << " Matched feature " << i << " in object image with feature " << match << " in image." << std::endl;
      /// adding feature-ROI
      ROI r( (int)(__img_features[matches[i]].x)-5, (int)(__img_features[matches[i]].y )-5, 11, 11, _width, _height);
      rv->push_back(r);
      /// increment feature-match-count
      ++c;
    }
  }
  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_matchin);
  //#endif
  std::cout << "SurfClassifier(classify) matched '" << c << "' of '" << __obj_features.size() << "' features in scene." << std::endl;


  std::cout << "SurfClassifier(classify): computing ROI" << std::endl;
  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(__ttc_roimerg);
  //#endif
  for (unsigned i = 0; i < matches.size(); i++) {
    if (matches[i] != -1) {
      // //(int)__obj_features[i].x, (int)__obj_features[i].y
      //(int)__img_features[matches[i]].x, (int)(__img_features[matches[i]].y );
      if( (int)__img_features[matches[i]].x < x_min )
	x_min = (int)__img_features[matches[i]].x;
      if( (int)__img_features[matches[i]].y < y_min )
	y_min = (int)__img_features[matches[i]].y;
      if( (int)__img_features[matches[i]].x > x_max )
	x_max = (int)__img_features[matches[i]].x;
      if( (int)__img_features[matches[i]].y > y_max )
	y_max = (int)__img_features[matches[i]].y;
    }
  }
  if( c != 0 ) {
    ROI r(x_min, y_min, x_max-x_min, y_max-y_min, _width, _height);
    rv->push_back(r);
  }
  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_roimerg);
  //#endif

  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(0);
  //#endif

  //#ifdef SURF_TIMETRACKER
  // print timetracker statistics
  __tt->print_to_stdout();
  //#endif

  std::cout << "SurfClassifier(classify): done ... returning '" << rv->size() << "' ROIs." << std::endl;
  return rv;
}

int
SurfClassifier::findMatch(const surf::Ipoint& ip1, const std::vector< surf::Ipoint >& ipts) {
  double mind = 1e100, second = 1e100;
  int match = -1;
  
  for (unsigned i = 0; i < ipts.size(); i++) {
    // Take advantage of Laplacian to speed up matching
    if (ipts[i].laplace != ip1.laplace)
      continue;
    
    double d = distSquare(ipts[i].ivec, ip1.ivec, __vlen);
    
    if (d < mind) {
      second = mind;
      mind = d;
      match = i;
    } else if (d < second) {
      second = d;
    }
  }
  
  if (mind < 0.5 * second)
    return match;
  
  return -1;
}


double
SurfClassifier::distSquare(double *v1, double *v2, int n) {
	double dsq = 0.;
	while (n--) {
		dsq += (*v1 - *v2) * (*v1 - *v2);
		v1++;
		v2++;
	}
	return dsq;
}
