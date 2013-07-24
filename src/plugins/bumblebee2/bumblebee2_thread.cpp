
/***************************************************************************
 *  bumblebee2_thread.cpp - Acquire data from Bumblebee2 stereo camera
 *
 *  Created: Wed Jul 17 13:17:27 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "bumblebee2_thread.h"

#include <pcl_utils/utils.h>
#include <pcl_utils/comparisons.h>
#include <utils/time/wait.h>
#include <utils/math/angle.h>
#include <fvcams/bumblebee2.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#include <interfaces/SwitchInterface.h>
#include <interfaces/OpenCVStereoParamsInterface.h>

#include <triclops.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace firevision;
using namespace fawkes;

#define CFG_PREFIX "/bumblebee2/"
#define CFG_OPENCV_PREFIX CFG_PREFIX"opencv-stereo/"

/** @class Bumblebee2Thread "bumblebee2_thread.h"
 * Thread to acquire data from Bumblebee2 stereo camera.
 * @author Tim Niemueller
 */

using namespace fawkes;

/// @cond INTERNALS
/** Data internal to Triclops stereo processor
 * This class exists to be able to hide the triclops stuff from the camera
 * user and not to expose the Triclops SDK headers.
 */
class TriclopsData
{
 public:
  TriclopsContext   context;
  TriclopsInput     input;
  TriclopsImage     rectified_image;
  TriclopsImage16   disparity_image_hires;
  TriclopsImage     disparity_image_lores;
  TriclopsImage3d * image_3d;
};
/// @endcond


/** Constructor. */
Bumblebee2Thread::Bumblebee2Thread()
  : Thread("Bumblebee2Thread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    TransformAspect(TransformAspect::ONLY_PUBLISHER, "Bumblebee2")
{
}


/** Destructor. */
Bumblebee2Thread::~Bumblebee2Thread()
{
}


void
Bumblebee2Thread::init()
{
  // prepare for finalize in the case init fails somewhere
  bb2_ = NULL;
  cv_disparity_ = NULL;
  tf_left_ = tf_right_ = NULL;
  switch_if_ = NULL;
  params_if_ = NULL;
  shm_img_rgb_right_ = shm_img_rgb_left_ = shm_img_yuv_right_ = shm_img_yuv_left_ = NULL;
  shm_img_rectified_right_ = shm_img_rectified_left_ = NULL;
  shm_img_prefiltered_right_ = shm_img_prefiltered_left_ = NULL;
  shm_img_rgb_rect_left_ = shm_img_rgb_rect_right_ = shm_img_disparity_ = NULL;
  triclops_ = NULL;
#ifdef USE_TIMETRACKER
  tt_ = NULL;
#endif

  // get config data
  cfg_base_frame_      = config->get_string(CFG_PREFIX"base-frame");
  cfg_frames_prefix_   = config->get_string(CFG_PREFIX"frames-prefix");
  cfg_frames_interval_ = config->get_float(CFG_PREFIX"frames-interval");

  std::string stereo_matcher = config->get_string(CFG_PREFIX"stereo-matcher");
  if (stereo_matcher == "opencv") {
    cfg_stereo_matcher_ = STEREO_MATCHER_OPENCV;
  } else if (stereo_matcher == "triclops") {
    cfg_stereo_matcher_ = STEREO_MATCHER_TRICLOPS;
  } else {
    throw Exception("Unknown stereo matcher %s", stereo_matcher.c_str());
  }

  // Open camera
  bb2_ = NULL;
  try {
    bb2_ = new Bumblebee2Camera();
    bb2_->open();
    bb2_->start();
    bb2_->set_image_number(Bumblebee2Camera::RGB_IMAGE);
  } catch (Exception &e) {
    finalize();
    throw;
  }

  // Open blackboard interfaces
  try {
    switch_if_ = blackboard->open_for_writing<SwitchInterface>("bumblebee2");
    switch_if_->set_enabled(true);
    switch_if_->write();
  } catch (Exception &e) {
    finalize();
    throw;
  }

  // allocate buffers
  width_  = bb2_->pixel_width();
  height_ = bb2_->pixel_height();

  buffer_rgb_       = bb2_->buffer();
  buffer_rgb_right_ = buffer_rgb_;
  buffer_rgb_left_  = buffer_rgb_right_ + colorspace_buffer_size(RGB, width_, height_); 
  buffer_green_     = (unsigned char *)malloc(width_ * height_ * 2);
  buffer_yuv_right_ = malloc_buffer(YUV422_PLANAR, width_, height_);
  buffer_yuv_left_  = malloc_buffer(YUV422_PLANAR, width_, height_);
  buffer_rgb_planar_right_ = malloc_buffer(RGB_PLANAR, width_, height_);
  buffer_rgb_planar_left_  = malloc_buffer(RGB_PLANAR, width_, height_);

  triclops_ = new TriclopsData();
  // Always the same
  triclops_->input.inputType   = TriInp_RGB;
  triclops_->input.nrows       = height_;
  triclops_->input.ncols       = width_;
  triclops_->input.rowinc      = triclops_->input.ncols;
  /*
  triclops_->input.u.rgb.red   = buffer_yuv_right;
  triclops_->input.u.rgb.green = buffer_yuv_left;
  triclops_->input.u.rgb.blue  = buffer_yuv_left;
  */
  triclops_->input.u.rgb.red   = buffer_green_;
  triclops_->input.u.rgb.green = buffer_green_ + width_ * height_;
  triclops_->input.u.rgb.blue  = triclops_->input.u.rgb.green;

  try {
    get_triclops_context_from_camera();
  } catch (Exception &e) {
    finalize();
    throw;
  }

  // We always need this for rectification
  triclopsSetNumberOfROIs(triclops_->context, 0);
  triclopsSetResolutionAndPrepare(triclops_->context,
				   height_, width_, height_, width_);


  TriclopsError err;

  err = triclopsGetImageCenter(triclops_->context, &center_row_, &center_col_);
  if (err != TriclopsErrorOk) {
    finalize();
    throw Exception("Failed to get image center: %s", triclopsErrorToString(err));
  }

  err = triclopsGetFocalLength(triclops_->context, &focal_length_);
  if (err != TriclopsErrorOk) {
    finalize();
    throw Exception("Failed to get focal length: %s", triclopsErrorToString(err));
  }

  err = triclopsGetBaseline(triclops_->context, &baseline_);
  if (err != TriclopsErrorOk) {
    finalize();
    throw Exception("Failed to get baseline: %s", triclopsErrorToString(err));
  }

  std::string stereo_frame = cfg_frames_prefix_;

  if (cfg_stereo_matcher_ == STEREO_MATCHER_TRICLOPS) {
    triclopsCreateImage3d(triclops_->context, &(triclops_->image_3d));

    // Set defaults
    triclopsSetSubpixelInterpolation(triclops_->context, 1);

    triclopsSetEdgeCorrelation(triclops_->context, 1);
    triclopsSetLowpass(triclops_->context, 1);
    triclopsSetDisparity(triclops_->context, 5, 100);
    triclopsSetEdgeMask(triclops_->context, 11);
    triclopsSetStereoMask(triclops_->context, 23);
    triclopsSetSurfaceValidation(triclops_->context, 1);
    triclopsSetTextureValidation(triclops_->context, 0);

    disparity_scale_factor_ = 1.0;

    stereo_frame += "right";

  } else if (cfg_stereo_matcher_ == STEREO_MATCHER_OPENCV) {
    // *** Read config values
    // pre-filtering (normalization of input images)
    try {
      std::string algorithm       = config->get_string(CFG_OPENCV_PREFIX"algorithm");
      if (algorithm == "bm") {
	cfg_opencv_stereo_algorithm_ = OPENCV_STEREO_BM;
      } else if (algorithm == "sgbm") {
	cfg_opencv_stereo_algorithm_ = OPENCV_STEREO_SGBM;
      } else {
	finalize();
	throw Exception("Unknown stereo algorithm '%s', use bm or sgbm", algorithm.c_str());
      }

      std::string pre_filter_type = config->get_string(CFG_OPENCV_PREFIX"pre-filter-type");
      cfg_bm_pre_filter_size_     = config->get_uint(CFG_OPENCV_PREFIX"pre-filter-size");
      cfg_bm_pre_filter_cap_      = config->get_uint(CFG_OPENCV_PREFIX"pre-filter-cap");

      // correspondence using Sum of Absolute Difference (SAD)
      cfg_bm_sad_window_size_     = config->get_uint(CFG_OPENCV_PREFIX"sad-window-size");
      cfg_bm_min_disparity_       = config->get_int(CFG_OPENCV_PREFIX"min-disparity");
      cfg_bm_num_disparities_     = config->get_uint(CFG_OPENCV_PREFIX"num-disparities");

      // post-filtering
      cfg_bm_texture_threshold_   = config->get_uint(CFG_OPENCV_PREFIX"texture-threshold");
      cfg_bm_uniqueness_ratio_    = config->get_uint(CFG_OPENCV_PREFIX"uniqueness-ratio");
      cfg_bm_speckle_window_size_ = config->get_uint(CFG_OPENCV_PREFIX"speckle-window-size");
      cfg_bm_speckle_range_       = config->get_uint(CFG_OPENCV_PREFIX"speckle-range");

      cfg_bm_try_smaller_windows_ = config->get_bool(CFG_OPENCV_PREFIX"try-smaller-windows");

      // SGBM specific values
      std::string sgbm_p1 = config->get_string(CFG_OPENCV_PREFIX"sgbm-p1");
      cfg_sgbm_p1_auto_ = (sgbm_p1 == "auto");
      if (! cfg_sgbm_p1_auto_) {
	cfg_sgbm_p1_              = config->get_int(CFG_OPENCV_PREFIX"sgbm-p1");
      }
      std::string sgbm_p2 = config->get_string(CFG_OPENCV_PREFIX"sgbm-p2");
      cfg_sgbm_p2_auto_ = (sgbm_p2 == "auto");
      if (! cfg_sgbm_p2_auto_) {
	cfg_sgbm_p2_              = config->get_int(CFG_OPENCV_PREFIX"sgbm-p1");
      }
      cfg_sgbm_disp_12_max_diff_  = config->get_int(CFG_OPENCV_PREFIX"sgbm-disp12-max-diff");

      // *** check config values
      if (pre_filter_type == "normalized_response") {
	cfg_bm_pre_filter_type_ = CV_STEREO_BM_NORMALIZED_RESPONSE;
      } else if (pre_filter_type == "xsobel") {
	cfg_bm_pre_filter_type_ = CV_STEREO_BM_XSOBEL;
      } else {
	throw Exception("Invalid OpenCV stereo matcher pre filter type");
      }

      if (cfg_bm_pre_filter_size_ < 5 || cfg_bm_pre_filter_size_ > 255 ||
	  cfg_bm_pre_filter_size_ % 2 == 0)
      {
	throw Exception("Pre filter size must be odd and be within 5..255");
      }
      
      if( cfg_bm_pre_filter_cap_ < 1 || cfg_bm_pre_filter_cap_ > 63 ) {
	throw Exception("Pre filter cap must be within 1..63");
      }

      if( cfg_bm_sad_window_size_ < 5 || cfg_bm_sad_window_size_ > 255 ||
	  cfg_bm_sad_window_size_ % 2 == 0 ||
	  cfg_bm_sad_window_size_ >= std::min(width_, height_))
      {
	throw Exception("SAD window size must be odd, be within 5..255 and "
			"be no larger than image width or height");
      }

      if( cfg_bm_num_disparities_ <= 0 || cfg_bm_num_disparities_ % 16 != 0 ) {
	throw Exception("Number of disparities must be positive and divisble by 16");
      }
    } catch (Exception &e) {
      finalize();
      throw;
    }

    int max_disparity =
      std::max((unsigned int)16, cfg_bm_min_disparity_ + cfg_bm_num_disparities_);
    logger->log_debug(name(), "Min Z: %fm  Max Z: %f",
		      focal_length_ * baseline_ / max_disparity,
		      (cfg_bm_min_disparity_ == 0)
		      ? std::numeric_limits<float>::infinity()
		      : focal_length_ * baseline_ / cfg_bm_min_disparity_);

    try {
      params_if_ = blackboard->open_for_writing<OpenCVStereoParamsInterface>("bumblebee2");
      switch (cfg_bm_pre_filter_type_) {
      case CV_STEREO_BM_NORMALIZED_RESPONSE:
	params_if_->set_pre_filter_type(OpenCVStereoParamsInterface::PFT_NORMALIZED_RESPONSE);
	break;
      case CV_STEREO_BM_XSOBEL:
	params_if_->set_pre_filter_type(OpenCVStereoParamsInterface::PFT_XSOBEL);
	break;
      default:
	throw Exception("No valid pre filter type set");
      }
      params_if_->set_pre_filter_size(cfg_bm_pre_filter_size_);
      params_if_->set_pre_filter_cap(cfg_bm_pre_filter_cap_);
      params_if_->set_sad_window_size(cfg_bm_sad_window_size_);
      params_if_->set_min_disparity(cfg_bm_min_disparity_);
      params_if_->set_num_disparities(cfg_bm_num_disparities_);
      params_if_->set_texture_threshold(cfg_bm_texture_threshold_);
      params_if_->set_uniqueness_ratio(cfg_bm_uniqueness_ratio_);
      params_if_->set_speckle_window_size(cfg_bm_speckle_window_size_);
      params_if_->set_speckle_range(cfg_bm_speckle_range_);
      params_if_->set_try_smaller_windows(cfg_bm_try_smaller_windows_);
      params_if_->write();
    } catch (Exception &e) {
      finalize();
      throw;
    }

    // We don't need this when using OpenCV
    triclopsSetEdgeCorrelation(triclops_->context, 0);
    triclopsSetLowpass(triclops_->context, 0);

    cv_disparity_ = new cv::Mat(height_, width_, CV_16SC1);
    // OpenCV disparity data is scaled by factor 16, always
    disparity_scale_factor_ = 1.f / 16.f;

    stereo_frame += "left";
  }

  try {
    pcl_xyz_ = new pcl::PointCloud<pcl::PointXYZ>();
    pcl_xyz_->is_dense = false;
    pcl_xyz_->width    = width_;
    pcl_xyz_->height   = height_;
    pcl_xyz_->points.resize(width_ * height_);
    pcl_xyz_->header.frame_id = stereo_frame;

    pcl_xyzrgb_ = new pcl::PointCloud<pcl::PointXYZRGB>();
    pcl_xyzrgb_->is_dense = false;
    pcl_xyzrgb_->width    = width_;
    pcl_xyzrgb_->height   = height_;
    pcl_xyzrgb_->points.resize(width_ * height_);
    pcl_xyzrgb_->header.frame_id = stereo_frame;

    pcl_manager->add_pointcloud("bumblebee2-xyz",    pcl_xyz_);
    pcl_manager->add_pointcloud("bumblebee2-xyzrgb", pcl_xyzrgb_);

    shm_img_rgb_right_ =
      new SharedMemoryImageBuffer("bumblebee2-rgb-right", RGB, width_, height_);
    shm_img_rgb_left_ =
      new SharedMemoryImageBuffer("bumblebee2-rgb-left", RGB, width_, height_);
    shm_img_yuv_right_ =
      new SharedMemoryImageBuffer("bumblebee2-yuv-right", YUV422_PLANAR, width_, height_);
    shm_img_yuv_left_ =
      new SharedMemoryImageBuffer("bumblebee2-yuv-left", YUV422_PLANAR, width_, height_);
    shm_img_rgb_rect_right_ =
      new SharedMemoryImageBuffer("bumblebee2-rgb-rectified-right", RGB_PLANAR, width_, height_);
    shm_img_rgb_rect_left_ =
      new SharedMemoryImageBuffer("bumblebee2-rgb-rectified-left", RGB_PLANAR, width_, height_);
    shm_img_rectified_right_ =
      new SharedMemoryImageBuffer("bumblebee2-rectified-right", MONO8, width_, height_);
    shm_img_rectified_left_ =
      new SharedMemoryImageBuffer("bumblebee2-rectified-left", MONO8, width_, height_);
    shm_img_prefiltered_right_ =
      new SharedMemoryImageBuffer("bumblebee2-prefiltered-right", MONO8, width_, height_);
    shm_img_prefiltered_left_ =
      new SharedMemoryImageBuffer("bumblebee2-prefiltered-left", MONO8, width_, height_);
    shm_img_disparity_ =
      new SharedMemoryImageBuffer("bumblebee2-disparity", MONO8, width_, height_);

    tf_last_publish_ = new fawkes::Time(clock);
    fawkes::Time now(clock);
    tf::Quaternion q(-M_PI/2.f, 0, -M_PI/2.f);
    tf::Transform t_left(q, tf::Vector3(0.0, 0.06, 0.018));
    tf::Transform t_right(q, tf::Vector3(0.0, -0.06, 0.018));

    tf_left_  =
      new tf::StampedTransform(t_left,  now, cfg_base_frame_, cfg_frames_prefix_ + "left");
    tf_right_ =
      new tf::StampedTransform(t_right, now, cfg_base_frame_, cfg_frames_prefix_ + "right");
  } catch (Exception &e) {
    finalize();
    throw;
  }

#ifdef USE_TIMETRACKER
  tt_ = new TimeTracker();
  tt_loopcount_ = 0;
  ttc_full_loop_    = tt_->add_class("Full Loop");
  ttc_transforms_   = tt_->add_class("Transforms");
  ttc_msgproc_      = tt_->add_class("Message Processing");
  ttc_capture_      = tt_->add_class("Capture");
  ttc_preprocess_   = tt_->add_class("Pre-processing");
  ttc_rectify_      = tt_->add_class("Rectification");
  ttc_stereo_match_ = tt_->add_class("Stereo Match");
  ttc_pcl_xyzrgb_   = tt_->add_class("PCL XYZRGB");
  ttc_pcl_xyz_      = tt_->add_class("PCL XYZ");
#endif
}


/** Get Triclops context.
 * This retrieves calibration information from the camera and stores it into a
 * temporary file. With this file the Triclops context is initialized. Afterwards
 * the temporary file is removed.
 */
void
Bumblebee2Thread::get_triclops_context_from_camera()
{
  char *tmpname = (char *)malloc(strlen("triclops_cal_XXXXXX") + 1);
  strcpy(tmpname, "triclops_cal_XXXXXX");
  char *tmpfile = mktemp(tmpname);
  bb2_->write_triclops_config_from_camera_to_file(tmpfile);

  TriclopsError err = triclopsGetDefaultContextFromFile(&(triclops_->context), tmpfile);
  if (err != TriclopsErrorOk) {
    free(tmpfile);
    throw Exception("Fetching Triclops context from camera failed");
  }
  unlink(tmpfile);
  free(tmpfile);
}

/** Deinterlace green buffer.
 * Method used in stereo processing. Following the PTGrey example, seems useless
 * if we have YUV planar and thus grey images anyway.
 * @param src source buffer
 * @param dest destination buffer
 * @param width width of the image
 * @param height height of the image
 */
void
Bumblebee2Thread::deinterlace_green(unsigned char* src, unsigned char* dest, 
				    unsigned int width, unsigned int height)
{
  register int i = (width*height)-2;
  register int g = ((width*height)/3)-1;

  while (i >= 0) {
    dest[g--] = src[i-=3];
  }
}

void
Bumblebee2Thread::finalize()
{
  delete tf_left_;
  delete tf_right_;

  pcl_manager->remove_pointcloud("bumblebee2-xyz");
  pcl_manager->remove_pointcloud("bumblebee2-xyzrgb");
  pcl_xyz_.reset();
  pcl_xyzrgb_.reset();
  
  blackboard->close(switch_if_);
  blackboard->close(params_if_);

  delete shm_img_rgb_right_;
  delete shm_img_rgb_left_;
  delete shm_img_yuv_right_;
  delete shm_img_yuv_left_;
  delete shm_img_rectified_right_;
  delete shm_img_rectified_left_;
  delete shm_img_prefiltered_right_;
  delete shm_img_prefiltered_left_;
  delete shm_img_rgb_rect_left_;
  delete shm_img_rgb_rect_right_;
  delete shm_img_disparity_;

  delete triclops_;
  delete cv_disparity_;

  if (buffer_green_)             free(buffer_green_);
  if (buffer_yuv_right_)         free(buffer_yuv_right_);
  if (buffer_yuv_left_)          free(buffer_yuv_left_);
  if (buffer_rgb_planar_right_)  free(buffer_rgb_planar_right_);
  if (buffer_rgb_planar_left_)   free(buffer_rgb_planar_left_);
  if (bb2_) {
    try {
      bb2_->stop();
      bb2_->close();
    } catch (Exception &e) {
      logger->log_warn(name(), "Stopping camera failed, exception follows");
      logger->log_warn(name(), e);
    }
    delete bb2_;
  }

#ifdef USE_TIMETRACKER
  delete tt_;
#endif
}


void
Bumblebee2Thread::loop()
{
  TIMETRACK_START(ttc_full_loop_);

  fawkes::Time now(clock);
  if ((now - tf_last_publish_) > cfg_frames_interval_) {
    TIMETRACK_START(ttc_transforms_);
    tf_last_publish_->stamp();

    // date time stamps slightly into the future so they are valid
    // for longer and need less frequent updates.
    fawkes::Time timestamp = now + (cfg_frames_interval_ * 1.1);

    tf_left_->stamp  = timestamp;
    tf_right_->stamp = timestamp;

    tf_publisher->send_transform(*tf_left_);
    tf_publisher->send_transform(*tf_right_);
    TIMETRACK_END(ttc_transforms_);
  }

  bool uses_triclops = (cfg_stereo_matcher_ == STEREO_MATCHER_TRICLOPS);
  bool uses_opencv   = (cfg_stereo_matcher_ == STEREO_MATCHER_OPENCV);


  TIMETRACK_START(ttc_msgproc_);

  if (uses_opencv) {
    while (! params_if_->msgq_empty()) {
      if (OpenCVStereoParamsInterface::SetPreFilterTypeMessage *msg =
	  params_if_->msgq_first_safe(msg))
      {
	switch (msg->pre_filter_type()) {
	case OpenCVStereoParamsInterface::PFT_NORMALIZED_RESPONSE:
	  cfg_bm_pre_filter_type_ = CV_STEREO_BM_NORMALIZED_RESPONSE;
	  break;
	case OpenCVStereoParamsInterface::PFT_XSOBEL:
	  cfg_bm_pre_filter_type_ = CV_STEREO_BM_XSOBEL;
	  break;
	}
	params_if_->set_pre_filter_type(msg->pre_filter_type());
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetPreFilterSizeMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_pre_filter_size_ = msg->pre_filter_size();
	params_if_->set_pre_filter_size(cfg_bm_pre_filter_size_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetPreFilterCapMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_pre_filter_cap_ = msg->pre_filter_cap();
	params_if_->set_pre_filter_cap(cfg_bm_pre_filter_cap_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetSADWindowSizeMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_sad_window_size_ = msg->sad_window_size();
	params_if_->set_sad_window_size(cfg_bm_sad_window_size_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetMinDisparityMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_min_disparity_ = msg->min_disparity();
	params_if_->set_min_disparity(cfg_bm_min_disparity_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetNumDisparitiesMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_num_disparities_ = msg->num_disparities();
	params_if_->set_num_disparities(cfg_bm_num_disparities_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetTextureThresholdMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_texture_threshold_ = msg->texture_threshold();
	params_if_->set_texture_threshold(cfg_bm_texture_threshold_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetUniquenessRatioMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_uniqueness_ratio_ = msg->uniqueness_ratio();
	params_if_->set_uniqueness_ratio(cfg_bm_uniqueness_ratio_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_speckle_window_size_ = msg->speckle_window_size();
	params_if_->set_speckle_window_size(cfg_bm_speckle_window_size_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetSpeckleRangeMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_speckle_range_ = msg->speckle_range();
	params_if_->set_speckle_range(cfg_bm_speckle_range_);
	params_if_->write();
      } else if (OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage *msg =
		 params_if_->msgq_first_safe(msg))
      {
	cfg_bm_try_smaller_windows_ = msg->is_try_smaller_windows();
	params_if_->set_try_smaller_windows(cfg_bm_try_smaller_windows_);
	params_if_->write();
      }

      params_if_->msgq_pop();
    }
  }

  while (! switch_if_->msgq_empty()) {
    if (SwitchInterface::EnableSwitchMessage *msg =
        switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(true);
      switch_if_->write();
    } else if (SwitchInterface::DisableSwitchMessage *msg =
               switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(false);
      switch_if_->write();
    }

    switch_if_->msgq_pop();
  }
  TIMETRACK_END(ttc_msgproc_);

  if (! switch_if_->is_enabled()) {
    TIMETRACK_ABORT(ttc_full_loop_);
    TimeWait::wait(250000);
    return;
  }

  TIMETRACK_START(ttc_capture_);

  // Acquire and process data
  bb2_->capture();
  fawkes::Time capture_ts(clock);

  TIMETRACK_INTER(ttc_capture_, ttc_preprocess_)

  bb2_->deinterlace_stereo();
  bb2_->decode_bayer();

  // Publish RGB and YUV images if requested
  if (shm_img_rgb_right_->num_attached() > 1) {
    shm_img_rgb_right_->lock_for_write();
    memcpy(shm_img_rgb_right_->buffer(), buffer_rgb_right_,
	   colorspace_buffer_size(RGB, width_, height_));
    shm_img_rgb_right_->set_capture_time(&capture_ts);
    shm_img_rgb_right_->unlock();
  }

  if (shm_img_rgb_left_->num_attached() > 1) {
    shm_img_rgb_left_->lock_for_write();
    memcpy(shm_img_rgb_left_->buffer(), buffer_rgb_left_,
	   colorspace_buffer_size(RGB, width_, height_));
    shm_img_rgb_left_->set_capture_time(&capture_ts);
    shm_img_rgb_left_->unlock();
  }

  if (shm_img_yuv_right_->num_attached() > 1) {
    shm_img_yuv_right_->lock_for_write();
    convert(RGB, YUV422_PLANAR, buffer_rgb_right_,
	    shm_img_yuv_right_->buffer(), width_, height_);
    shm_img_yuv_right_->set_capture_time(&capture_ts);
    shm_img_yuv_right_->unlock();
  }

  if (shm_img_yuv_left_->num_attached() > 1) {
    shm_img_yuv_left_->lock_for_write();
    convert(RGB, YUV422_PLANAR, buffer_rgb_left_,
	    shm_img_yuv_left_->buffer(), width_, height_);
    shm_img_yuv_left_->set_capture_time(&capture_ts);
    shm_img_yuv_left_->unlock();
  }

  TriclopsError err;

  // Extract green buffer and rectify image
  deinterlace_green(buffer_rgb_, buffer_green_, width_, 6 * height_);

  TIMETRACK_INTER(ttc_preprocess_, ttc_rectify_);

  err = triclopsRectify(triclops_->context, &(triclops_->input));
  if (err != TriclopsErrorOk) {
    logger->log_warn(name(), "Rectifying the image failed (%s), skipping loop",
		     triclopsErrorToString(err));
    bb2_->dispose_buffer();
    return;
  }

  // get rectified images and publish if requested
  TriclopsImage image_right, image_left;
  err = triclopsGetImage(triclops_->context, TriImg_RECTIFIED, TriCam_RIGHT, &image_right);
  if (err != TriclopsErrorOk) {
    logger->log_warn(name(), "Retrieving right rectified image failed (%s), skipping loop",
		     triclopsErrorToString(err));
    bb2_->dispose_buffer();
    return;
  }
  err = triclopsGetImage(triclops_->context, TriImg_RECTIFIED, TriCam_LEFT, &image_left);
  if (err != TriclopsErrorOk) {
    logger->log_warn(name(), "Retrieving left rectified image failed (%s), skipping loop",
		     triclopsErrorToString(err));
    bb2_->dispose_buffer();
    return;
  }

  if (shm_img_rectified_right_->num_attached() > 1) {
    shm_img_rectified_right_->lock_for_write();
    memcpy(shm_img_rectified_right_->buffer(), image_right.data,
	   colorspace_buffer_size(MONO8, width_, height_));
    shm_img_rectified_right_->set_capture_time(&capture_ts);
    shm_img_rectified_right_->unlock();
  }
  if (shm_img_rectified_left_->num_attached() > 1) {
    shm_img_rectified_left_->lock_for_write();
    memcpy(shm_img_rectified_left_->buffer(), image_left.data,
	   colorspace_buffer_size(MONO8, width_, height_));
    shm_img_rectified_left_->set_capture_time(&capture_ts);
    shm_img_rectified_left_->unlock();
  }

  TIMETRACK_INTER(ttc_rectify_, ttc_stereo_match_);

  // stereo correspondence matching
  short int *dispdata = NULL;
  if (uses_triclops) {
    err = triclopsStereo(triclops_->context);
    if (err != TriclopsErrorOk) {
      logger->log_warn(name(), "Calculating the disparity image failed (%s), skipping loop",
		       triclopsErrorToString(err));
      bb2_->dispose_buffer();
      return;
    }

    triclopsGetImage16(triclops_->context, TriImg16_DISPARITY, TriCam_REFERENCE,
		       &(triclops_->disparity_image_hires));
    dispdata = (short int *)triclops_->disparity_image_hires.data;

  } else if (uses_opencv) {
    // Get Images and wrap with OpenCV data structures
    cv::Mat img_r(height_, width_, CV_8UC1, image_right.data);
    cv::Mat img_l(height_, width_, CV_8UC1, image_left.data);

    // Calculate disparity
    if (cfg_opencv_stereo_algorithm_ == OPENCV_STEREO_BM) {
      cv::StereoBM block_matcher(cv::StereoBM::BASIC_PRESET,
				 cfg_bm_num_disparities_, cfg_bm_sad_window_size_);
      block_matcher.state->preFilterType     = cfg_bm_pre_filter_type_;
      block_matcher.state->preFilterSize     = cfg_bm_pre_filter_size_;
      block_matcher.state->preFilterCap      = cfg_bm_pre_filter_cap_;
      block_matcher.state->minDisparity      = cfg_bm_min_disparity_;
      block_matcher.state->textureThreshold  = cfg_bm_texture_threshold_;
      block_matcher.state->uniquenessRatio   = cfg_bm_uniqueness_ratio_;
      block_matcher.state->speckleWindowSize = cfg_bm_speckle_window_size_;
      block_matcher.state->speckleRange	   = cfg_bm_speckle_range_;
      block_matcher.state->trySmallerWindows = cfg_bm_try_smaller_windows_ ? 1 : 0;

      block_matcher(img_l, img_r, *cv_disparity_);

      if (shm_img_prefiltered_right_->num_attached() > 1) {
	shm_img_prefiltered_right_->lock_for_write();
	memcpy(shm_img_prefiltered_right_->buffer(),
	       block_matcher.state->preFilteredImg0->data.ptr,
	       colorspace_buffer_size(MONO8, width_, height_));
	shm_img_prefiltered_right_->set_capture_time(&capture_ts);
	shm_img_prefiltered_right_->unlock();
      }
      if (shm_img_prefiltered_left_->num_attached() > 1) {
	shm_img_prefiltered_left_->lock_for_write();
	memcpy(shm_img_prefiltered_left_->buffer(),
	       block_matcher.state->preFilteredImg0->data.ptr,
	       colorspace_buffer_size(MONO8, width_, height_));
	shm_img_prefiltered_left_->set_capture_time(&capture_ts);
	shm_img_prefiltered_left_->unlock();
      }
    } else {
      int cn = img_l.channels();

      cv::StereoSGBM block_matcher;
      block_matcher.minDisparity = cfg_bm_min_disparity_;
      block_matcher.numberOfDisparities = cfg_bm_num_disparities_;
      block_matcher.SADWindowSize = cfg_bm_sad_window_size_;
      block_matcher.preFilterCap = cfg_bm_pre_filter_cap_;
      block_matcher.uniquenessRatio = cfg_bm_uniqueness_ratio_;
      if (cfg_sgbm_p1_auto_) {
	block_matcher.P1 =  8 * cn * block_matcher.SADWindowSize * block_matcher.SADWindowSize;
      } else {
	block_matcher.P1 = cfg_sgbm_p1_;
      }
      if (cfg_sgbm_p2_auto_) {
	block_matcher.P2 = 32 * cn * block_matcher.SADWindowSize * block_matcher.SADWindowSize;
      } else {
	block_matcher.P2 = cfg_sgbm_p2_;
      }
      if (block_matcher.P1 >= block_matcher.P2) {
	logger->log_warn(name(), "SGBM P1 >= P2 (%i <= %i), skipping loop",
			 block_matcher.P1, block_matcher.P2);
	bb2_->dispose_buffer();
	return;
      }

      block_matcher.speckleWindowSize = cfg_bm_speckle_window_size_;
      block_matcher.speckleRange = cfg_bm_speckle_range_;
      block_matcher.disp12MaxDiff = 1;
      block_matcher. fullDP = false;

      block_matcher(img_l, img_r, *cv_disparity_);
    }

    dispdata = (short int *)(cv_disparity_->data);
  }

  if (shm_img_disparity_->num_attached() > 1) {
    cv::Mat normalized_disparity(height_, width_, CV_16SC1);
    cv::Mat original_disparity(height_, width_, uses_triclops ? CV_16UC1 : CV_16SC1, dispdata);
    cv::normalize(original_disparity, normalized_disparity, 0, 256, cv::NORM_MINMAX);
    shm_img_disparity_->lock_for_write();
    unsigned char *buffer = shm_img_disparity_->buffer();
    for (unsigned int i = 0; i < width_ * height_; ++i) {
      buffer[i] = (unsigned char)((short int *)(normalized_disparity.data))[i];
    }
    shm_img_disparity_->set_capture_time(&capture_ts);
    shm_img_disparity_->unlock();
  }

  TIMETRACK_END(ttc_stereo_match_);

  // 2 is us and the PCL manager of the PointCloudAspect
  bool want_xyzrgb   = (pcl_xyzrgb_.use_count() > 2);
  bool want_xyz      = (pcl_xyz_.use_count() > 2);

  TriclopsColorImage img_right_rect_color;
  if (shm_img_rgb_rect_right_->num_attached() > 1 || (want_xyzrgb && uses_triclops)) {
    convert(RGB, RGB_PLANAR, buffer_rgb_right_, buffer_rgb_planar_right_ , width_, height_);
    TriclopsInput img_rgb_right;
    img_rgb_right.inputType = TriInp_RGB;
    img_rgb_right.nrows = height_;
    img_rgb_right.ncols = width_;
    img_rgb_right.rowinc = width_;
    img_rgb_right.u.rgb.red   = buffer_rgb_planar_right_;
    img_rgb_right.u.rgb.green = buffer_rgb_planar_right_ + (width_ * height_    );
    img_rgb_right.u.rgb.blue  = buffer_rgb_planar_right_ + (width_ * height_ * 2);
    err = triclopsRectifyColorImage(triclops_->context, TriCam_RIGHT,
				    &img_rgb_right, &img_right_rect_color);
    if (err != TriclopsErrorOk) {
      logger->log_warn(name(), "Rectifying right color image failed (%s), skipping loop",
		       triclopsErrorToString(err));
      bb2_->dispose_buffer();
      return;
    }
    if (shm_img_rgb_rect_right_->num_attached() > 1) {
      shm_img_rgb_rect_right_->lock_for_write();
      memcpy(shm_img_rgb_rect_right_->buffer(), img_right_rect_color.red, width_ * height_);
      memcpy(shm_img_rgb_rect_right_->buffer() + width_ * height_,
	     img_right_rect_color.green, width_ * height_);
      memcpy(shm_img_rgb_rect_right_->buffer() + width_ * height_ * 2,
	     img_right_rect_color.blue, width_ * height_);
      shm_img_rgb_rect_right_->set_capture_time(&capture_ts);
      shm_img_rgb_rect_right_->unlock();
    }
  }

  TriclopsColorImage img_left_rect_color;
  if (shm_img_rgb_rect_left_->num_attached() > 1 || (want_xyzrgb && uses_opencv)) {
    convert(RGB, RGB_PLANAR, buffer_rgb_left_, buffer_rgb_planar_left_ , width_, height_);
    TriclopsInput img_rgb_left;
    img_rgb_left.inputType = TriInp_RGB;
    img_rgb_left.nrows = height_;
    img_rgb_left.ncols = width_;
    img_rgb_left.rowinc = width_;
    img_rgb_left.u.rgb.red   = buffer_rgb_planar_left_;
    img_rgb_left.u.rgb.green = buffer_rgb_planar_left_ + (width_ * height_    );
    img_rgb_left.u.rgb.blue  = buffer_rgb_planar_left_ + (width_ * height_ * 2);
    err = triclopsRectifyColorImage(triclops_->context, TriCam_LEFT,
				    &img_rgb_left, &img_left_rect_color);
    if (err != TriclopsErrorOk) {
      logger->log_warn(name(), "Rectifying left color image failed (%s), skipping loop",
		       triclopsErrorToString(err));
      bb2_->dispose_buffer();
      return;
    }
    if (shm_img_rgb_rect_left_->num_attached() > 1) {
      shm_img_rgb_rect_left_->lock_for_write();
      memcpy(shm_img_rgb_rect_left_->buffer(), img_left_rect_color.red, width_ * height_);
      memcpy(shm_img_rgb_rect_left_->buffer() + width_ * height_,
	     img_left_rect_color.green, width_ * height_);
      memcpy(shm_img_rgb_rect_left_->buffer() + width_ * height_ * 2,
	     img_left_rect_color.blue, width_ * height_);
      shm_img_rgb_rect_left_->set_capture_time(&capture_ts);
      shm_img_rgb_rect_left_->unlock();
    }
  }

  // Triclops' reference image is the right camera, OpenCV uses left
  TriclopsColorImage *img_reference_rect_color =
    (uses_triclops) ? &img_right_rect_color : &img_left_rect_color;

  // Calculate 3D point cloud from data
  pcl::PointCloud<pcl::PointXYZ>    &pcl_xyz    = **pcl_xyz_;
  pcl::PointCloud<pcl::PointXYZRGB> &pcl_xyzrgb = **pcl_xyzrgb_;

  if (want_xyz && want_xyzrgb) {
    fill_xyz_xyzrgb(dispdata, img_reference_rect_color, pcl_xyz, pcl_xyzrgb);

    pcl_xyz.header.seq += 1;
    pcl_xyzrgb.header.seq += 1;
    pcl_utils::set_time(pcl_xyz_, capture_ts);
    pcl_utils::set_time(pcl_xyzrgb_, capture_ts);
  } else if (want_xyz) {
    fill_xyz(dispdata, pcl_xyz);
    pcl_xyz.header.seq += 1;
    pcl_utils::set_time(pcl_xyz_, capture_ts);
  } else if (want_xyzrgb) {
    fill_xyzrgb(dispdata, img_reference_rect_color, pcl_xyzrgb);
    pcl_xyzrgb.header.seq += 1;
    pcl_utils::set_time(pcl_xyzrgb_, capture_ts);
  }

  bb2_->dispose_buffer();

  TIMETRACK_END(ttc_full_loop_);

#ifdef USE_TIMETRACKER
  if ((++tt_loopcount_ % 30) == 0) {
    tt_->print_to_stdout();
  }
  if (tt_loopcount_ >= 150) {
    tt_loopcount_ = 0;
    tt_->reset();
  }
#endif

}


// Methods to fill in point clouds
// Z = fB/d
// where
// Z = distance along the camera Z axis
// f = focal length (in pixels)
// B = baseline (in metres)
// d = disparity (in pixels) 
// After Z is determined, X and Y can be calculated using
// the usual projective camera equations:
// 
// X = uZ/f
// Y = vZ/f

// where
// u and v are the pixel location in the 2D image
// X, Y, Z is the real 3d position

// Note: u and v are not the same as row and column. You must
// account for the image center. You can get the image center using
// the triclopsGetImageCenter() function. Then you find u and v by:

// u = col - centerCol
// v = row - centerRow

// Note: If u, v, f, and d are all in pixels and X,Y,Z are all in
// the meters, the units will always work i.e. pixel/pixel =
// no-unit-ratio = m/m.


void
Bumblebee2Thread::fill_xyz_xyzrgb(const short int *dispdata,
				  const TriclopsColorImage *img_rect_color,
				  pcl::PointCloud<pcl::PointXYZ> &pcl_xyz,
				  pcl::PointCloud<pcl::PointXYZRGB> &pcl_xyzrgb)
{
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < height_; ++h) {
    for (unsigned int w = 0; w < width_; ++w, ++idx, ++dispdata) {

      pcl::PointXYZ &xyz = pcl_xyz.points[idx];
      pcl::PointXYZRGB &xyzrgb = pcl_xyzrgb.points[idx];

      float d = *dispdata * disparity_scale_factor_;
      if (d <= cfg_bm_min_disparity_) {
	xyz.x = xyz.y = xyz.z = xyzrgb.x = xyzrgb.y = xyzrgb.z = bad_point;
	continue;
      }

      float b_by_d = baseline_/d;
      xyz.z = xyzrgb.z = focal_length_ * b_by_d;
      xyz.x = xyzrgb.x = ((float)w - center_col_) * b_by_d;
      xyz.y = xyzrgb.y = ((float)h - center_row_) * b_by_d;

      xyzrgb.r = img_rect_color->red[idx];
      xyzrgb.g = img_rect_color->green[idx];
      xyzrgb.b = img_rect_color->blue[idx];
    }
  }
}


void
Bumblebee2Thread::fill_xyzrgb(const short int *dispdata,
			      const TriclopsColorImage *img_rect_color,
			      pcl::PointCloud<pcl::PointXYZRGB> &pcl_xyzrgb)
{
  TIMETRACK_START(ttc_pcl_xyzrgb_);
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < height_; ++h) {
    for (unsigned int w = 0; w < width_; ++w, ++idx, ++dispdata) {

      pcl::PointXYZRGB &xyzrgb = pcl_xyzrgb.points[idx];

      float d = *dispdata * disparity_scale_factor_;
      if (d <= cfg_bm_min_disparity_) {
	xyzrgb.x = xyzrgb.y = xyzrgb.z = bad_point;
	continue;
      }

      float b_by_d = baseline_/d;
      xyzrgb.z = focal_length_ * b_by_d;
      xyzrgb.x = ((float)w - center_col_) * b_by_d;
      xyzrgb.y = ((float)h - center_row_) * b_by_d;

      xyzrgb.r = img_rect_color->red[idx];
      xyzrgb.g = img_rect_color->green[idx];
      xyzrgb.b = img_rect_color->blue[idx];
    }
  }
  TIMETRACK_END(ttc_pcl_xyzrgb_);
}

void
Bumblebee2Thread::fill_xyz(const short int *dispdata,
			   pcl::PointCloud<pcl::PointXYZ> &pcl_xyz)
{
  TIMETRACK_START(ttc_pcl_xyz_);
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < height_; ++h) {
    for (unsigned int w = 0; w < width_; ++w, ++idx, ++dispdata) {

      pcl::PointXYZ &xyz = pcl_xyz.points[idx];

      // OpenCV disparity data is scaled by factor 16, always
      float d = *dispdata * disparity_scale_factor_;
      if (d <= cfg_bm_min_disparity_) {
	xyz.x = xyz.y = xyz.z = bad_point;
	continue;
      }

      float b_by_d = baseline_/d;
      xyz.z = focal_length_ * b_by_d;
      xyz.x = ((float)w - center_col_) * b_by_d;
      xyz.y = ((float)h - center_row_) * b_by_d;
    }
  }
  TIMETRACK_END(ttc_pcl_xyz_);
}
