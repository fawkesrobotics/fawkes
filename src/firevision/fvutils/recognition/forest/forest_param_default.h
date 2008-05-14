/***************************************************************************
 *  forest_param_default default values for parameters  
 *
 *  Created: Wed Dec 12 13:04:12 2008
 *  Copyright  2008  Vaishak Belle
 *
 * 
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




#ifndef __FIREVISION_FVUTILS_RECOGNITION_FOREST_FOREST_PARAM_DEFAULT_H_
#define __FIREVISION_FVUTILS_RECOGNITION_FOREST_FOREST_PARAM_DEFAULT_H_


// Features Related
#define RECTANGULAR_FEATURE_SIZE 8 // assume the faces are scaled to 24x24
#define RECTANGULAR_MIN_FEATURE_SIZE 2 // we do not pixel operations or boxes of 2x2 etc. This sets the minimum size for the boxes. 
#define HAARFEATURES 4
#define NCLASSES 2

// Growth of Tree
#define NITERATIONS 100
#define NODECOUNT 400
#define MAX_TRY_ALLOWED 20 // maximum number of tries the children is allowed to sent back to tree growth tests : _deprecated_ 

#define _FOREST 10

#define NOS_SCALES 11 //How many differnet scales of the images to try out sliding window?

// Modes of running Program  
#define RUN_CLASSIFICATION false //Classification Mode
#define RUN_SLIDING_WINDOW true // Detection mode
#define VERBOSE true 

#define LEARN_OFFLINE false // Learning Entire tree from saved data (!! not implemented completely !!)

#define DISCARD_SMALL true //discard little rectangles that show up - conne3cted to training size
#define DISCARD_SMALL_DIMENSION 30


/* SUBWINDOW_RANGE decides whether every possible subwindow location should be sampled, or whether every
 * alternate pixel should be sampled 
 * or every third pixel should be sampled and so on 
 */
#define SUBWINDOW_RANGE 2 // How exhaustively should the subwindows be searched: 

#define ALLOW_CROP_BOUNDARY_HEURISTIC false //allow a few pixels to the left and top to define the face rather than the original bounding box
#define ALLOW_WITSCALING false // This decides of the parameter should be individually scaled for each scale factor
/* ( Integration threshold for points identified as neighbors )
 * Since I am using just a simple non-intelligent combine votes from the per scale detections and only apply a strong 
 * merging when I have fully collected config.forestWindows(), NEIGHBOR_RANGE IS COMPLETELY USELESS! -Nov23,2007
 */
/*deprecated*/ #define NEIGHBOR_RANGE 5 // DEPRECATED
/*deprecated*/ #define DETECTION_THRESHOLD 20 // What is the range between whcih detections are consdiered identical


#define AREA_OF_SUBWINDOW_MERGE 0.3  // 0.0005 // same as area of merge but for mergeSubdwindows (detections of same scale)

//#define WINDOWS_INTEGRATION_THRESHOLD 19  // Integration threshold for "merge" of subsequent neighbor hits
//#define FILTER_NOISE_THRESHOLD 4 // Filtering out smaller rectangles inside/around bigger ones
//#define AREA_OF_MERGE 0.2 // Check UserDef::filterNoise2

#define FACE_MAP_THRESHOLD 0.2 // Default Face map threshold value (fmp)
#define IDENTITY_THRESHOLD 0.2 // identity threshold for face recognition
#define NEIGHBOR_RANGE 5 //the range for which filterNoise should search for neighbors 

#define TEST_IMAGE_SLIDING_WINDOW "../../IMAGES/CMU/newtest/jpegs/nens.jpg" // Test image for sliding window

#define FINAL_DETECTIONS "../info/final_detections.dat"

#define TRAIN_IMAGES_LOC "/home/Diplomanden/vaishak/Code/image_databases/"
//#define TRAIN_IMAGES_LOC "../../IMAGES/MITCBCL/train/" // Loc of train images

#define TEST_IMAGES_LOC "../../IMAGES/MITCBCL/train/" // Loc of test images (for classification task)

#define TRAINDT "../info/train" // Loc of training images' integral images
#define TESTDT "../info/test.dat" // Loc of test images' integral images

#define TRAINTESTS "../info/traintests.dat"  

#define TRAINTREE "../info/traintree.dat"

#define CONFIGFILE "../info/config.dat" // Loc of config file

#define TREEDOT "../results/tree.dot" // Loc of graphviz output of tree grown (Not for forest!)


#define NOS_THRESHOLDS 1 // Number of thresholds to use for feature Pass

#define WR_FACE_MAP_FILE false 
#define WR_DOT_FILE false


#define ERR_NWORKING printf("\n This module is not working. \n")
#define ERR_MODULE(x) printf("\n The module %s is one of the follows: (1) not implemented (2) implementation not checked \n", x )


#endif

