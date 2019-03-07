
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

#ifndef _FIREVISION_CLASSIFIERS_SURF_H_
#define _FIREVISION_CLASSIFIERS_SURF_H_

#ifndef HAVE_SURF
#	error SURF not available, you may not use the SurfClassifier
#endif

#include <fvclassifiers/classifier.h>

#include <vector>

// FIXME replace with forward declarations
#include <surf/image.h>
#include <surf/ipoint.h>
//class surf::Ipoint;
//class surf::Image;
#include <utils/time/clock.h>
#include <utils/time/tracker.h>

//#define NUM_OBJ 13
#define OFFLINE_SURF true // offline reading - reading from descriptors folder
#define MIN_MATCH_RATIO 0.05

//#ifdef SURF_TIMETRACKER
namespace fawkes {
class TimeTracker;
}
//#endif

//struct CvMemStorage;
//typedef struct _IplImage IplImage;

namespace firevision {

void saveIpoints(std::string                      sFileName,
                 const std::vector<surf::Ipoint> &ipts,
                 bool                             bVerbose,
                 bool                             bLaplacian,
                 int                              VLength);

void loadIpoints(std::string sFileName, std::vector<surf::Ipoint> &ipts, bool bVerbose, int &);

class SurfClassifier : public Classifier
{
public:
	//instantiating using descriptor files
	SurfClassifier(std::string  keypoints_descriptor_txt_file,
	               unsigned int min_match       = 5,
	               float        min_match_ratio = MIN_MATCH_RATIO,
	               int          samplingStep    = 2,
	               int          octaves         = 4,
	               double       thres           = 4.0,
	               bool         doubleImageSize = false,
	               int          initLobe        = 3,
	               bool         upright         = false,
	               bool         extended        = false,
	               int          indexSize       = 4);

	//instantiating using a directory containing the png images
	SurfClassifier(const char * image_directory_png_files,
	               unsigned int min_match       = 5,
	               float        min_match_ratio = MIN_MATCH_RATIO,
	               int          samplingStep    = 2,
	               int          octaves         = 4,
	               double       thres           = 4.0,
	               bool         doubleImageSize = false,
	               int          initLobe        = 3,
	               bool         upright         = false,
	               bool         extended        = false,
	               int          indexSize       = 4);

	virtual ~SurfClassifier();

	virtual std::list<ROI> *classify();

private:
	unsigned int num_obj_; // number of objects

	// Find closest interest point in a list, given one interest point
	int findMatch(const surf::Ipoint &ip1, const std::vector<surf::Ipoint> &ipts);

	// Calculate square distance of two vectors
	double distSquare(double *v1, double *v2, int n);

	// Object objects
	surf::Image *                          obj_img_;
	std::vector<std::vector<surf::Ipoint>> obj_features_;
	std::vector<std::string>               obj_names_;
	int                                    obj_num_features_;

	// Image objects
	surf::Image *             image_;
	std::vector<surf::Ipoint> img_features_;
	int                       img_num_features_;

	// minimum (absolute) number of features that have to be matched per ROI
	unsigned int min_match_;
	// minimum ratio of features per total object-features that have to be matched per ROI
	float min_match_ratio_;

	// Initial sampling step (default 2)
	int samplingStep_;
	// Number of analysed octaves (default 4)
	int octaves_;
	// Blob response treshold
	double thres_;
	// Set this flag "true" to double the image size
	bool doubleImageSize_;
	// Initial lobe size, default 3 and 5 (with double image size)
	int initLobe_;
	// Upright SURF or rotation invaraiant
	bool upright_;
	// If the extended flag is turned on, SURF 128 is used
	bool extended_;
	// Spatial size of the descriptor window (default 4)
	int indexSize_;

	// Length of descriptor vector
	int vlen_;

	//#ifdef SURF_TIMETRACKER
	fawkes::TimeTracker *tt_;
	unsigned int         loop_count_;
	unsigned int         ttc_objconv_;
	unsigned int         ttc_objfeat_;
	unsigned int         ttc_imgconv_;
	unsigned int         ttc_imgfeat_;
	unsigned int         ttc_matchin_;
	unsigned int         ttc_roimerg_;
	//#endif
};

} // end namespace firevision

#endif
