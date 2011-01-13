
/***************************************************************************
 *  surf.cpp - SURF based classifier
 *
 *  Created: Tue Apr 01 10:15:23 2008
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

#include <fvclassifiers/surf.h>
#include <math.h>
//#ifdef SURF_TIMETRACKER
#include <utils/time/clock.h>
#include <utils/time/tracker.h>
//#endif
#include <fstream>

#include <string>

#include <surf/surflib.h>
//#include <surf/ipoint.h>
//#include <surf/image.h>
//#include <surf/imload.h>

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <fvutils/readers/png.h>
#include <utils/system/console_colors.h>
#include <dirent.h>
#include <utils/logging/liblogger.h>

#define BVERBOSE true

//#include <fvutils/writers/pnm.h>
//#include <fvutils/writers/png.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SurfClassifier <fvclassifiers/surf.h>
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


/** saveIpoints
 * save Surf points
 * @param sFileName surf file name
 * @param ipts surf ipoints (surf::iPoint)
 * @param bVerbose verbose mode
 * @param bLaplacian laplacian mode
 */

void saveIpoints(std::string sFileName, const std::vector< surf::Ipoint >& ipts, bool bVerbose, bool bLaplacian, int VLength)
{
  std::cout<<"Attempting to save interest points" << std::endl;

  std::ofstream ipfile(sFileName.c_str());
  if( !ipfile ) {
    std::cerr << "ERROR in loadIpoints(): "
	      << "Couldn't open file '" << sFileName.c_str() << "'!" << std::endl; //STS
    return;
  }

  double sc;
  unsigned count = ipts.size();

  // Write the file header
  if (bLaplacian)
     ipfile << VLength + 1 << std::endl << count << std::endl;
   else
     ipfile << VLength << std::endl << count << std::endl;
  // In order to just save the interest points without descriptor, comment
  // the above and uncomment the following command.
  //  ipfile << 1.0 << std::endl << count << std::endl;
  // Save interest point with descriptor in the format of Krystian Mikolajczyk
  // for reasons of comparison with other descriptors. As our interest points
  // are circular in any case, we use the second component of the ellipse to
  // provide some information about the strength of the interest point. This is
  // important for 3D reconstruction as only the strongest interest points are
  // considered. Replace the strength with 0.0 in order to perform Krystian's
  // comparisons.
  for (unsigned n=0; n<ipts.size(); n++){
    // circular regions with diameter 5 x scale
    sc = 2.5 * ipts[n].scale; sc*=sc;
    ipfile  << ipts[n].x /* x-location of the interest point */
            << " " << ipts[n].y /* y-location of the interest point */
            << " " << 1.0/sc /* 1/r^2 */
            << " " << 0.0     //(*ipts)[n]->strength /* 0.0 */
            << " " << 1.0/sc; /* 1/r^2 */

    if (bLaplacian)
      ipfile << " " << ipts[n].laplace;

    // Here comes the descriptor
    for (int i = 0; i < VLength; i++) {
      ipfile << " " << ipts[n].ivec[i];
    }
    ipfile << std::endl;
  }

  // Write message to terminal.
  if( bVerbose )
    std::cout << count << " interest points found" << std::endl;
}

/** loadIpoints
 * load interest points
 * @param sFileName location of the interest points
 * @param ipts vector to store interest points
 * @param bVerbose if the saveIpoints was carried out with verbose mode
 */
void loadIpoints( std::string sFileName, std::vector< surf::Ipoint >& ipts, bool bVerbose, int& __vlen )
{
  std::ifstream ipfile(sFileName.c_str());

  if( !ipfile ) {
  std::cerr << "ERROR in loadIpoints(): "
	    << "Couldn't open file '" << sFileName.c_str() << "'!" << std::endl; //STS
    return;
  }

  // Load the file header

  unsigned count;
  ipfile >> __vlen >> count;

  __vlen--;

  // create a new interest point vector
  ipts.clear();
  ipts.resize(count);

  // Load the interest points in Mikolajczyk's format
  for (unsigned n=0; n<count; n++){
    // circular regions with diameter 5 x scale
    float x, y, a, b, c;
    ipfile >> x >> y >> a >> b >> c;

    float det = sqrt((a-c)*(a-c) + 4.0*b*b);
    float e1 = 0.5*(a+c + det);
    float e2 = 0.5*(a+c - det);
    float l1 = (1.0/sqrt(e1));
    float l2 = (1.0/sqrt(e2));
    float sc = sqrt( l1*l2 );

    ipts[n].x     = x;
    ipts[n].y     = y;
    ipts[n].scale = sc/2.5;
    ipfile >> ipts[n].laplace;

    //ipts[n].allocIvec( VLength );
    ipts[n].ivec = new double[ __vlen];

    for( int j = 0 ; j < __vlen; j++ )
      {

	ipfile >> ipts[n].ivec[j];

	//	std::cout << ipts[n].ivec[j] << " ";
      }

  }

  // close the interest point file again
  ipfile.close();

  // Write message to terminal.
  if( bVerbose )
    std::cout << "read in " << count << " interest points." << std::endl;
}

/** Constructor.
 * @param keypoints_dir location of the keypoints (descriptor file as a txt file) for the reference objects
 * @param samplingStep Initial sampling step
 * @param min_match minimum number of features that have to be matched per ROI
 * @param min_match_ratio  minimum ratio of features matched per object to be matched per ROI
 * @param octaves Number of analysed octaves
 * @param thres Blob response treshold
 * @param doubleImageSize true to double the image size, false to keep original
 * @param initLobe Initial lobe size, default 3 and 5 (with double image size)
 * @param upright rotation invariance (fasle) or upright (true)
 * @param extended true to use the extended descriptor (SURF 128)
 * @param indexSize Spatial size of the descriptor window (default 4)

 */
SurfClassifier::SurfClassifier( std::string keypoints_dir, unsigned int min_match, float min_match_ratio,
				int samplingStep, int octaves, double thres,
				bool doubleImageSize, int initLobe,
				bool upright, bool extended, int indexSize ): Classifier("SurfClassifier")
{
  __obj_features.clear();
  __obj_features.reserve(1000);
  // matching constraints
  __min_match = min_match;
  __min_match_ratio = min_match_ratio;
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
  __tt = new fawkes::TimeTracker();
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

  DIR *dir = 0;

  if( (dir = opendir( keypoints_dir.c_str() ) ) == NULL ) {
	  char* buffer = new char[256];
      sprintf(buffer, "The directory %s does not exist!", keypoints_dir.c_str() );
      fawkes::LibLogger::log_error("SurfClassifier",buffer);
  }

  struct dirent* ent;
  std::string object_file;
  int num_obj_index = 0;


  while( (ent = readdir(dir)) != NULL ) {

      if ( strcmp( ent->d_name, ".") == 0 || strcmp( ent->d_name,"..") == 0 || strcmp( ent->d_name,".svn") == 0 )
    	  continue;

      object_file = keypoints_dir + ent->d_name;
      std:: cout<<"SurfClassifier: reading the following descriptor file" << object_file << std::endl;

      __obj_names.push_back(object_file);


      bool b_verbose = BVERBOSE;
      loadIpoints( object_file, __obj_features[num_obj_index], b_verbose, __vlen);
      num_obj_index++;

    }

  closedir(dir);
  delete ent;

  __num_obj = num_obj_index;

  if( num_obj_index != 0 ) {
	  std::cout<< "SurfClassifier: Reading successful"<< std::endl;
	  //#ifdef SURF_TIMETRACKER
	  __tt->ping_end(__ttc_objconv);
	  //#endif
  }
  else {
// if no objects were read, then the descriptor files were probably not created still. We can create them now!
	  std::cout <<"SurfClassifier: The descriptor directory is probably empty since no objects were read off. Will instantiate a Surfclassifier with the png images directory") << std::endl;
	  return new SurfClassifier( "../res/opx/objects/", 5 );
  }


  // save object image for debugging
  ///surf::ImLoad::saveImage( "obj.pgm", __obj_img);

  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(__ttc_objfeat);
  //#endif
 //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_objfeat);
  //#endif


}





/** Constructor.
 * @param object_dir file that contains an image of the object to detect
 * @param samplingStep Initial sampling step
 * @param min_match minimum number of features that have to be matched per ROI
 * @param min_match_ratio  minimum ratio of features matched per object to be matched per ROI
 * @param octaves Number of analysed octaves
 * @param thres Blob response treshold
 * @param doubleImageSize true to double the image size, false to keep original
 * @param initLobe Initial lobe size, default 3 and 5 (with double image size)
 * @param upright rotation invariance (fasle) or upright (true)
 * @param extended true to use the extended descriptor (SURF 128)
 * @param indexSize Spatial size of the descriptor window (default 4)
 */


SurfClassifier::SurfClassifier( const char * object_dir,
				unsigned int min_match, float min_match_ratio,
				int samplingStep, int octaves, double thres,
				bool doubleImageSize, int initLobe,
				bool upright, bool extended, int indexSize)
  : Classifier("SurfClassifier")
{

  __obj_features.clear();
  __obj_features.reserve(1000);
  // matching constraints
  __min_match = min_match;
  __min_match_ratio = min_match_ratio;
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
  __tt = new fawkes::TimeTracker();
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


  DIR *dir = 0;

  std::string dir_path = object_dir;

  if( (dir = opendir( dir_path.c_str() ) ) == NULL )
    {
      char* buffer = new char[256];
      sprintf(buffer, "The directory %s does not exist!", dir_path.c_str() );

      fawkes::LibLogger::log_error("SurfClassifier",buffer);
    }

  struct dirent* ent;
  std::string object_file;
  int num_obj_index = 0;

  while( (ent = readdir(dir)) != NULL ) {

	  if ( strcmp( ent->d_name, ".") == 0 || strcmp( ent->d_name,"..") == 0 || strcmp( ent->d_name,".svn") == 0)
		  continue;

	  object_file = dir_path + ent->d_name;

//       if( !object_file && strcmp( object_file, "" ) == 0 ) {
//     throw fawkes::Exception("empty object file");
//   }

  std::cout << "SurfClassifier(classify): opening object image file '" << object_file << "'" << std::endl;

  PNGReader pngr( object_file.c_str() );
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
    throw fawkes::Exception("Could not load object file '%s'", object_file.c_str());
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

  std::vector<surf::Ipoint> obj_feature;
  __obj_features.push_back( obj_feature );
  __obj_features[num_obj_index].clear();
  __obj_features[num_obj_index].reserve(1000);
  __obj_num_features = 0;
  // Extract interest points with Fast-Hessian
  surf::FastHessian fh(__obj_img, /* pointer to integral image */
		       __obj_features[num_obj_index],
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
  for (unsigned n=0; n < __obj_features[num_obj_index].size(); n++){
    // set the current interest point
    des.setIpoint(&(__obj_features.at(num_obj_index).at(n)));
    // assign reproducible orientation
    des.assignOrientation();
    // make the SURF descriptor
    des.makeDescriptor();

  }


  __obj_num_features = __obj_features[num_obj_index].size();
  if ( ! __obj_num_features > 0 ) {
    throw fawkes::Exception("Could not compute object features");
  }
  std::cout << "SurfClassifier(classify): computed '" << __obj_num_features << "' features from object" << std::endl;

  char buffer[256];
  sprintf( buffer, "descriptors/%s-%d.surf", ent->d_name, num_obj_index );
  std::string des_file_name = buffer;

  bool b_verbose = BVERBOSE;
  bool b_laplacian = true;

  __obj_names.push_back( des_file_name );


  // save descriptor
  saveIpoints( des_file_name, __obj_features[num_obj_index], b_verbose, b_laplacian, __vlen );


  // CleanUp
  delete __simage;

  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_objfeat);
  //#endif

  num_obj_index++;
    }

  __num_obj = num_obj_index;

}


/** Destructor. */
SurfClassifier::~SurfClassifier()
{
  //
}


std::list< ROI > *
SurfClassifier::classify()
{

  //  std::cout<<"SurfClassifier: Entering classification:-"<< std::endl;
  //#ifdef SURF_TIMETRACKER
  __tt->ping_start(0);
  //#endif

  // list of ROIs to return

  std::list<ROI> rv[__num_obj];
  float match_ratios[__num_obj];


  //  std::list< ROI > *rv = new std::list< ROI >();

  // for ROI calculation
  int x_min = _width;
  int y_min = _height;
  int x_max = 0;
  int y_max = 0;

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
  std::cout<<"surfclassifer/classify : getting interest points"<<std::endl;
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

  for( unsigned j = 0; j < __num_obj; j++ )
    {
      std::vector< int > matches(__obj_features[j].size());
      //      std::cout<< "SurfClassifier; _debug_ : " << __obj_features[j].size()  << "and" << __img_features.size() << std::endl;
      int c = 0;
      for (unsigned i = 0; i < __obj_features[j].size(); i++) {
	int match = findMatch((__obj_features[j])[i], __img_features);
	matches[i] = match;
	if (match != -1) {
	  // std::cout << " Matched feature " << i << " in object image with feature " << match << " in image." << std::endl;
	  /// adding feature-ROI
	  ROI r( (int)(__img_features[matches[i]].x)-5, (int)(__img_features[matches[i]].y )-5, 11, 11, _width, _height);
	  r.num_hint_points = 0;
	  rv[j].push_back(r);
	  /// increment feature-match-count
	  ++c;
	}
      }
      //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_matchin);
  //#endif
  if( c == 0 )
  std::cout << "SurfClassifier(classify) matched '" << c << fawkes::cnormal <<"' of '" << __obj_features[j].size() << "' features in scene. (for supplied object = " << j << std::endl ;
  else
  std::cout << "SurfClassifier(classify) matched '" << fawkes::cblue << c << fawkes::cnormal <<"' of '" << __obj_features[j].size() << "' features in scene. (for supplied object = " << j << std::endl ;


  float match_ratio = ((float)c / (float)__obj_features[j].size());
  match_ratios[j] = match_ratio;

  std::cout << "SurfClassifier(classify): match_ratio is '" << match_ratio << "' and min_match_ratio is" << __min_match_ratio << std::endl;

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
  if( (c != 0) && ((unsigned)c > __min_match) &&
      (match_ratio > __min_match_ratio) &&
      (x_max - x_min != 0 ) && (y_max - y_min != 0) ) {

    std::cout << "SurfClassifier(classify): c='" << c << "' __min_match='" << __min_match << "'." << std::endl;

    ROI r(x_min, y_min, x_max-x_min, y_max-y_min, _width, _height);
    r.num_hint_points = c;
    rv[j].push_back(r);
   } else {
    std::cout << " clearing ROI-list (no or too few matches or [0,0]-roi!)" << std::endl;
    rv[j].clear();
  }
    }
  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(__ttc_roimerg);
  //#endif

  // CleanUp
  delete __image;
  delete __simage;

  //#ifdef SURF_TIMETRACKER
  __tt->ping_end(0);
  //#endif

  //#ifdef SURF_TIMETRACKER
  // print timetracker statistics
  //__tt->print_to_stdout();
  //#endif


  // histogram comparison of all rois and features detected
  float min_ratio_tmp = -1.0;
  int min_ratio_index = -1;
  for( unsigned int i = 0; i < __num_obj; i++ )
    {
      if( match_ratios[i] > min_ratio_tmp )
	{
	  min_ratio_tmp = match_ratios[i];
	  min_ratio_index = i;
	}
    }

  std::list<ROI> *final_rv = new std::list<ROI>;

  final_rv->assign( rv[min_ratio_index].begin(), rv[min_ratio_index].end() );


  std::string first_not(".-");
  int first_not_index = __obj_names[ min_ratio_index ].find_first_of( first_not );
  std::string obj_name_tmp( __obj_names[ min_ratio_index ] );
  obj_name_tmp.erase( first_not_index );


  std::cout << "SurfClassifier(classify): done,  ... returning '" << rv->size() << "' ROIs. The object class is " << min_ratio_index << "and object name is " << fawkes::cgreen << obj_name_tmp << fawkes::cnormal << std::endl;
  return final_rv;
}

int
SurfClassifier::findMatch(const surf::Ipoint& ip1, const std::vector< surf::Ipoint >& ipts) {
  double mind = 1e100, second = 1e100;
  int match = -1;

  //  std::cout<< "SurfClassifier/findMatch: " << ipts.size() <<" " << __vlen << std::endl;

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
  //  std::cout<< fawkes::cblue << (*v1) << fawkes::cred << (*v2);

  while (n--) {

    dsq += (*v1 - *v2) * (*v1 - *v2);
    v1++;
    v2++;
  }

  //  std::cout << fawkes::cgreen << " "<<dsq << std::endl;

  return dsq;
}

} // end namespace firevision
