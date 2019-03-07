
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

#include <fvclassifiers/siftpp.h>

#include <iostream>
#include <vector>

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
SiftppClassifier::SiftppClassifier(const char *object_file,
                                   int         samplingStep,
                                   int         octaves,
                                   int         levels,
                                   float       magnif,
                                   int         noorient,
                                   int         unnormalized)
: Classifier("SiftppClassifier")
{
	// params for FastHessian
	samplingStep_ = samplingStep;
	octaves_      = octaves;
	levels_       = levels;
	// params for Descriptors
	first_         = -1;
	threshold_     = 0.04f / levels_ / 2.0f;
	edgeThreshold_ = 10.0f;
	magnif_        = magnif;
	noorient_      = noorient;
	unnormalized_  = unnormalized;

	// descriptor vector length
	vlen_ = 128;

	//#ifdef SIFTPP_TIMETRACKER
	tt_          = new TimeTracker();
	loop_count_  = 0;
	ttc_objconv_ = tt_->add_class("ObjectConvert");
	ttc_objfeat_ = tt_->add_class("ObjectFeatures");
	ttc_imgconv_ = tt_->add_class("ImageConvert");
	ttc_imgfeat_ = tt_->add_class("ImageFeatures");
	ttc_matchin_ = tt_->add_class("Matching");
	ttc_roimerg_ = tt_->add_class("MergeROIs");
	//#endif

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_start(ttc_objconv_);
	//#endif

	PNGReader      pngr(object_file);
	unsigned char *buf = malloc_buffer(pngr.colorspace(), pngr.pixel_width(), pngr.pixel_height());
	pngr.set_buffer(buf);
	pngr.read();

	unsigned int lwidth  = pngr.pixel_width();
	unsigned int lheight = pngr.pixel_height();
	VL::pixel_t *im_pt   = new VL::pixel_t[lwidth * lheight];
	VL::pixel_t *start   = im_pt;
	//VL::pixel_t* end   = start + lwidth*lheight ;
	for (unsigned int h = 0; h < lheight; ++h) {
		for (unsigned int w = 0; w < lwidth; ++w) {
			int         i    = (buf[h * lwidth + w]);
			VL::pixel_t norm = VL::pixel_t(255);
			*start++         = VL::pixel_t(i) / norm;
		}
	}
	// make image
	obj_img_         = new VL::PgmBuffer();
	obj_img_->width  = lwidth;
	obj_img_->height = lheight;
	obj_img_->data   = im_pt;

	if (!obj_img_) {
		throw Exception("Could not load object file");
	}

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_end(ttc_objconv_);
	//#endif

	// save object image for debugging
	//

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_start(ttc_objfeat_);
	//#endif

	// COMPUTE OBJECT FEATURES
	obj_features_.clear();
	//obj_features_.reserve(1000);
	obj_num_features_ = 0;

	sigman_ = .5;
	sigma0_ = 1.6 * powf(2.0f, 1.0f / levels_);

	std::cout << "SiftppClassifier(ctor): init scalespace" << std::endl;
	// initialize scalespace
	VL::Sift sift(obj_img_->data,
	              obj_img_->width,
	              obj_img_->height,
	              sigman_,
	              sigma0_,
	              octaves_,
	              levels_,
	              first_,
	              -1,
	              levels_ + 1);

	std::cout << "SiftppClassifier(ctor): detect object keypoints" << std::endl;
	// Run SIFTPP detector
	sift.detectKeypoints(threshold_, edgeThreshold_);
	// Number of keypoints
	obj_num_features_ = sift.keypointsEnd() - sift.keypointsBegin();
	std::cout << "SiftppClassifier(ctor): computed '" << obj_num_features_ << "' object-keypoints"
	          << std::endl;

	// set descriptor options
	sift.setNormalizeDescriptor(!unnormalized_);
	sift.setMagnification(magnif_);

	std::cout << "SiftppClassifier(ctor): run detector, compute ori and des ..." << std::endl;
	// Run detector, compute orientations and descriptors
	for (VL::Sift::KeypointsConstIter iter = sift.keypointsBegin(); iter != sift.keypointsEnd();
	     ++iter) {
		//Feature * feat = new Feature();
		Feature feat;

		//std::cout << "SiftppClassifier(ctor): saving keypoint" << std::endl;
		feat.key = (*iter);

		// detect orientations
		VL::float_t angles[4];
		int         nangles;
		if (!noorient_) {
			nangles = sift.computeKeypointOrientations(angles, *iter);
		} else {
			nangles   = 1;
			angles[0] = VL::float_t(0);
		}
		feat.number_of_desc = nangles;
		feat.descs          = new VL::float_t *[nangles];

		//std::cout << "SiftppClassifier(ctor): computing '" << nangles << "' descriptors" << std::endl;
		// compute descriptors
		for (int a = 0; a < nangles; ++a) {
			//       out << setprecision(2) << iter->x << ' ' << setprecision(2) << iter->y << ' '
			// 	  << setprecision(2) << iter->sigma << ' ' << setprecision(3) << angles[a] ;
			// compute descriptor
			feat.descs[a] = new VL::float_t[vlen_];
			sift.computeKeypointDescriptor(feat.descs[a], *iter, angles[a]);
		} // next angle
		//std::cout << "SiftppClassifier(ctor): computed '" << feat.number_of_desc << "' descriptors." << std::endl;

		// save feature
		obj_features_.push_back(feat);

	} // next keypoint

	obj_num_features_ = obj_features_.size();
	if (!obj_num_features_ > 0) {
		throw Exception("Could not compute object features");
	}
	std::cout << "SiftppClassifier(ctor): computed '" << obj_num_features_ << "' features from object"
	          << std::endl;

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_end(ttc_objfeat_);
	//#endif
}

/** Destructor. */
SiftppClassifier::~SiftppClassifier()
{
	//
	delete obj_img_;
	obj_features_.clear();
	//
	//delete image_;
	img_features_.clear();
}

std::list<ROI> *
SiftppClassifier::classify()
{
	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_start(0);
	//#endif

	// list of ROIs to return
	std::list<ROI> *rv = new std::list<ROI>();

	// for ROI calculation
	int x_min = _width;
	int y_min = _height;
	int x_max = 0;
	int y_max = 0;

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_start(ttc_imgconv_);
	//#endif
	std::cout << "SiftppClassifier(classify): copy imgdat to SIFTPP Image" << std::endl;

	VL::pixel_t *im_pt = new VL::pixel_t[_width * _height];
	VL::pixel_t *start = im_pt;
	for (unsigned int h = 0; h < _height; ++h) {
		for (unsigned int w = 0; w < _width; ++w) {
			int         i    = (_src[h * _width + w]);
			VL::pixel_t norm = VL::pixel_t(255);
			*start++         = VL::pixel_t(i) / norm;
		}
	}
	// make image
	image_         = new VL::PgmBuffer();
	image_->width  = _width;
	image_->height = _height;
	image_->data   = im_pt;

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_end(ttc_imgconv_);
	//#endif

	/// Write image to verify correct operation
	// nothing yet

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_start(ttc_imgfeat_);
	//#endif

	// COMPUTE IMAGE FEATURES
	img_features_.clear();
	img_num_features_ = 0;
	//img_features_.reserve(1000);

	std::cout << "SiftppClassifier(classify): init scalespace" << std::endl;
	// initialize scalespace
	VL::Sift sift(image_->data,
	              image_->width,
	              image_->height,
	              sigman_,
	              sigma0_,
	              octaves_,
	              levels_,
	              first_,
	              -1,
	              levels_ + 1);

	std::cout << "SiftppClassifier(classify): detect image keypoints" << std::endl;
	// Run SIFTPP detector
	sift.detectKeypoints(threshold_, edgeThreshold_);

	// Number of keypoints
	img_num_features_ = sift.keypointsEnd() - sift.keypointsBegin();
	std::cout << "SiftppClassifier(classify): Extracted '" << img_num_features_ << "' image keypoints"
	          << std::endl;

	// set descriptor options
	sift.setNormalizeDescriptor(!unnormalized_);
	sift.setMagnification(magnif_);

	std::cout << "SiftppClassifier(classify): run detector, compute ori and des ..." << std::endl;
	// Run detector, compute orientations and descriptors
	for (VL::Sift::KeypointsConstIter iter = sift.keypointsBegin(); iter != sift.keypointsEnd();
	     ++iter) {
		Feature feat; // = new Feature();

		//std::cout << "SiftppClassifier(classify): saving keypoint" << std::endl;
		feat.key = (*iter);

		//std::cout << "SiftppClassifier(classify): detect orientations" << std::endl;
		// detect orientations
		VL::float_t angles[4];
		int         nangles;
		if (!noorient_) {
			nangles = sift.computeKeypointOrientations(angles, *iter);
		} else {
			nangles   = 1;
			angles[0] = VL::float_t(0);
		}
		feat.number_of_desc = nangles;
		feat.descs          = new VL::float_t *[nangles];

		//std::cout << "SiftppClassifier(classify): computing '" << nangles << "' descriptors" << std::endl;
		// compute descriptors
		for (int a = 0; a < nangles; ++a) {
			// compute descriptor
			feat.descs[a] = new VL::float_t[vlen_];
			sift.computeKeypointDescriptor(feat.descs[a], *iter, angles[a]);
		} // next angle
		//std::cout << "SiftppClassifier(classify): computed '" << feat.number_of_desc << "' descriptors." << std::endl;

		// save feature
		img_features_.push_back(feat);

	} // next keypoint

	// Number of feature
	img_num_features_ = img_features_.size();

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_end(ttc_imgfeat_);
	//#endif

	std::cout << "SiftppClassifier(classify): Extracted '" << img_num_features_ << "' image features"
	          << std::endl;

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_start(ttc_matchin_);
	//#endif
	std::cout << "SiftppClassifier(classify): matching ..." << std::endl;

	std::vector<int> matches(obj_features_.size());
	int              m = 0;
	for (unsigned i = 0; i < obj_features_.size(); i++) {
		int match  = findMatch(obj_features_[i], img_features_);
		matches[i] = match;
		if (match != -1) {
			std::cout << "SiftppClassifier(classify): Matched feature " << i
			          << " in object image with feature " << match << " in image." << std::endl;
			/// adding feature-ROI
			ROI r((int)(img_features_[matches[i]].key.x) - 5,
			      (int)(img_features_[matches[i]].key.y) - 5,
			      11,
			      11,
			      _width,
			      _height);
			rv->push_back(r);
			// increment feature-match-count
			++m;
		}
	}

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_end(ttc_matchin_);
	//#endif
	std::cout << "SiftppClassifier(classify) matched '" << m << "' of '" << obj_features_.size()
	          << "' features in scene." << std::endl;

	std::cout << "SiftppClassifier(classify): computing ROI" << std::endl;
	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_start(ttc_roimerg_);
	//#endif

	for (unsigned i = 0; i < matches.size(); i++) {
		if (matches[i] != -1) {
			if ((int)img_features_[matches[i]].key.x < x_min)
				x_min = (int)img_features_[matches[i]].key.x;
			if ((int)img_features_[matches[i]].key.y < y_min)
				y_min = (int)img_features_[matches[i]].key.y;
			if ((int)img_features_[matches[i]].key.x > x_max)
				x_max = (int)img_features_[matches[i]].key.x;
			if ((int)img_features_[matches[i]].key.y > y_max)
				y_max = (int)img_features_[matches[i]].key.y;
		}
	}
	if (m != 0) {
		ROI r(x_min, y_min, x_max - x_min, y_max - y_min, _width, _height);
		rv->push_back(r);
	}

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_end(ttc_roimerg_);
	//#endif

	//#ifdef SIFTPP_TIMETRACKER
	tt_->ping_end(0);
	//#endif

	//#ifdef SIFTPP_TIMETRACKER
	// print timetracker statistics
	tt_->print_to_stdout();
	//#endif

	delete image_;

	std::cout << "SiftppClassifier(classify): done ... returning '" << rv->size() << "' ROIs."
	          << std::endl;
	return rv;
}

int
SiftppClassifier::findMatch(const Feature &ip1, const std::vector<Feature> &ipts)
{
	double mind = 1e100, second = 1e100;
	int    match = -1;

	for (unsigned i = 0; i < ipts.size(); i++) {
		if (ipts[i].number_of_desc != ip1.number_of_desc)
			continue;
		//std::cout << "SiftppClassifier(findMatch): number_of_desc matched!" << std::endl;
		for (int j = 0; j < ip1.number_of_desc; ++j) {
			double d = distSquare(ipts[i].descs[j], ip1.descs[j], vlen_);

			if (d < mind) {
				second = mind;
				mind   = d;
				match  = i;
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
SiftppClassifier::distSquare(VL::float_t *v1, VL::float_t *v2, int n)
{
	double dsq = 0.;
	while (n--) {
		dsq += (v1[n - 1] - v2[n - 1]) * (v1[n - 1] - v2[n - 1]);
	}
	//std::cout << "  dsq: '" << dsq << "'" << std::endl;
	return dsq;
}

} // end namespace firevision
