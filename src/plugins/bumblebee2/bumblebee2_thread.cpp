
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

#include <interfaces/SwitchInterface.h>

#include <triclops.h>
#ifdef USE_OPENCV_STEREO
#  include <opencv2/core/core.hpp>
#  include <opencv2/calib3d/calib3d.hpp>
#endif

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
class TriclopsStereoProcessorData
{
 public:
  TriclopsContext   context;
  TriclopsInput     input;
  TriclopsError     err;
  TriclopsImage     rectified_image;
  TriclopsImage16   disparity_image_hires;
  TriclopsImage     disparity_image_lores;
  TriclopsImage3d * image_3d;
  bool              enable_subpixel_interpolation;
};
/// @endcond


/** Constructor. */
Bumblebee2Thread::Bumblebee2Thread()
  : Thread("Bumblebee2Thread", Thread::OPMODE_CONTINUOUS),
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
  bb2_ = NULL;
  try {
    bb2_ = new Bumblebee2Camera();
    bb2_->open();
    bb2_->start();
    bb2_->set_image_number(Bumblebee2Camera::RGB_IMAGE);
  } catch (Exception &e) {
    delete bb2_;
    throw;
  }

  try {
    switch_if_ = NULL;
    switch_if_ = blackboard->open_for_writing<SwitchInterface>("tabletop-objects");
    switch_if_->set_enabled(true);
    switch_if_->write();
  } catch (Exception &e) {
    blackboard->close(switch_if_);
    delete bb2_;
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

  triclops_ = new TriclopsStereoProcessorData();
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
    bb2_->stop();
    bb2_->close();
    delete bb2_;
    blackboard->close(switch_if_);
    delete bb2_;
    delete triclops_;
    throw;
  }

  // We always need this for rectification
  triclopsSetNumberOfROIs(triclops_->context, 0);
  triclopsSetResolutionAndPrepare(triclops_->context,
				   height_, width_, height_, width_);

#ifdef USE_TRICLOPS_STEREO
  triclopsCreateImage3d(triclops_->context, &(triclops_->image_3d));

  // Set defaults
  triclops_->enable_subpixel_interpolation = false;

  triclopsSetSubpixelInterpolation(triclops_->context, 0);

  triclopsSetEdgeCorrelation(triclops_->context, 1);
  triclopsSetLowpass(triclops_->context, 1);
  triclopsSetDisparity(triclops_->context, 5, 100);
  triclopsSetEdgeMask(triclops_->context, 11);
  triclopsSetStereoMask(triclops_->context, 23);
  triclopsSetSurfaceValidation(triclops_->context, 1);
  triclopsSetTextureValidation(triclops_->context, 0);
#elif defined(USE_OPENCV_STEREO)
  // *** Read config values
  // pre-filtering (normalization of input images)
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

  cfg_bm_try_smaller_widows_  = config->get_bool(CFG_OPENCV_PREFIX"try-smaller-windows");

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
      cfg_bm_sad_window_size_ % 2 == 0 || cfg_bm_sad_window_size_ >= std::min(width_, height_))
  {
    throw Exception("SAD window size must be odd, be within 5..255 and "
		    "be no larger than image width or height");
  }

  if( cfg_bm_num_disparities_ <= 0 || cfg_bm_num_disparities_ % 16 != 0 ) {
    throw Exception("Number of disparities must be positive and divisble by 16");
  }

  // We don't need this when using OpenCV
  triclopsSetEdgeCorrelation(triclops_->context, 0);
  triclopsSetLowpass(triclops_->context, 0);

#endif

  pcl_xyz_ = new pcl::PointCloud<pcl::PointXYZ>();
  pcl_xyz_->is_dense = false;
  pcl_xyz_->width    = width_;
  pcl_xyz_->height   = height_;
  pcl_xyz_->points.resize(width_ * height_);
  pcl_xyz_->header.frame_id = "/kinect/image";

  pcl_xyzrgb_ = new pcl::PointCloud<pcl::PointXYZRGB>();
  pcl_xyzrgb_->is_dense = false;
  pcl_xyzrgb_->width    = width_;
  pcl_xyzrgb_->height   = height_;
  pcl_xyzrgb_->points.resize(width_ * height_);
  pcl_xyzrgb_->header.frame_id = "/kinect/image";

  pcl_manager->add_pointcloud("bumblebee2-xyz",    pcl_xyz_);
  pcl_manager->add_pointcloud("bumblebee2-xyzrgb", pcl_xyzrgb_);

  shm_img_right_ = new SharedMemoryImageBuffer("bumblebee2-rgb-right", RGB, width_, height_);
  shm_img_left_ = new SharedMemoryImageBuffer("bumblebee2-rgb-left", RGB, width_, height_);
  shm_img_right_rectified_ =
    new SharedMemoryImageBuffer("bumblebee2-rectified-right", MONO8, width_, height_);
  shm_img_left_rectified_ =
    new SharedMemoryImageBuffer("bumblebee2-rectified-left", MONO8, width_, height_);
  shm_img_right_prefiltered_ =
    new SharedMemoryImageBuffer("bumblebee2-prefiltered-right", MONO8, width_, height_);
  shm_img_left_prefiltered_ =
    new SharedMemoryImageBuffer("bumblebee2-prefiltered-left", MONO8, width_, height_);
  shm_img_disparity_ =
    new SharedMemoryImageBuffer("bumblebee2-disparity", MONO8, width_, height_);

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

  triclops_->err = triclopsGetDefaultContextFromFile(&(triclops_->context), tmpfile);
  if (triclops_->err != TriclopsErrorOk) {
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
  pcl_manager->remove_pointcloud("bumblebee2-xyz");
  pcl_manager->remove_pointcloud("bumblebee2-xyzrgb");
  pcl_xyz_.reset();
  pcl_xyzrgb_.reset();
  
  blackboard->close(switch_if_);

  delete shm_img_right_;
  delete shm_img_left_;
  delete shm_img_right_rectified_;
  delete shm_img_left_rectified_;
  delete shm_img_disparity_;

  delete triclops_;
  free(buffer_green_);
  free(buffer_yuv_right_);
  free(buffer_yuv_left_);
  try {
    bb2_->stop();
    bb2_->close();
  } catch (Exception &e) {
    logger->log_warn(name(), "Stopping camera failed, exception follows");
    logger->log_warn(name(), e);
  }
  delete bb2_;
}


void
Bumblebee2Thread::loop()
{
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

  if (! switch_if_->is_enabled()) {
    TimeWait::wait(250000);
    return;
  }

  bb2_->capture();
  fawkes::Time capture_ts(clock);

  bb2_->deinterlace_stereo();
  bb2_->decode_bayer();

  if (shm_img_right_->num_attached() > 1) {
    shm_img_right_->lock_for_write();
    memcpy(shm_img_right_->buffer(), buffer_rgb_right_,
	   colorspace_buffer_size(RGB, width_, height_));
    shm_img_right_->set_capture_time(&capture_ts);
    shm_img_right_->unlock();
  }

  if (shm_img_left_->num_attached() > 1) {
    shm_img_left_->lock_for_write();
    memcpy(shm_img_left_->buffer(), buffer_rgb_left_,
	   colorspace_buffer_size(RGB, width_, height_));
    shm_img_left_->set_capture_time(&capture_ts);
    shm_img_left_->unlock();
  }

  // Can be optional depending on whether the image has actually been acquired or not
  //convert(RGB, YUV422_PLANAR, buffer_rgb_right, buffer_yuv_right, _width, _height);
  //convert(RGB, YUV422_PLANAR, buffer_rgb_left, buffer_yuv_left, _width, _height);

  deinterlace_green(buffer_rgb_, buffer_green_, width_, 6 * height_);

  triclops_->err = triclopsRectify(triclops_->context, &(triclops_->input));
  if (triclops_->err != TriclopsErrorOk) {
    logger->log_warn(name(), "Rectifying the image failed (%s), skipping loop",
		     triclopsErrorToString(triclops_->err));
    return;
  }

#ifdef USE_TRICLOPS_STEREO
  triclops_->err = triclopsStereo(triclops_->context);
  if (triclops_->err != TriclopsErrorOk) {
    logger->log_warn(name(), "Calculating the disparity image failed (%s), skipping loop",
		     triclopsErrorToString(triclops_->err));
    return;
  }

  triclopsExtractImage3d(triclops_->context, triclops_->image_3d);

  TriclopsImage3d *image_3d = triclops_->image_3d;
  pcl::PointCloud<pcl::PointXYZ> &pcl = **pcl_xyz_;
  pcl.header.seq += 1;
  pcl_utils::set_time(pcl_xyz_, capture_ts);

  unsigned int num_points = image_3d->nrows *image_3d->ncols;
  for(unsigned int idx = 0; idx < num_points; ++idx ) {
    TriclopsPoint3d &input = image_3d->points[idx];
    pcl::PointXYZ &output = pcl.points[idx];

    output.x = input.point[0];
    output.y = input.point[1];
    output.z = input.point[2];
  }

  /*
  if ( triclops_->enable_subpixel_interpolation ) {
    triclopsGetImage16(triclops_->context, TriImg16_DISPARITY, TriCam_REFERENCE,
		       &(triclops_->disparity_image_hires));
  } else {
    triclopsGetImage(triclops_->context, TriImg_DISPARITY, TriCam_REFERENCE,
		     &(triclops_->disparity_image_lores));
  }
  */

#elif defined(USE_OPENCV_STEREO)
  // Get Images and wrap with OpenCV data structures
  TriclopsImage image_right, image_left;
  TriclopsError err;
  err = triclopsGetImage(triclops_->context, TriImg_RECTIFIED, TriCam_RIGHT, &image_right);
  if (err != TriclopsErrorOk) {
    logger->log_warn(name(), "Retrieving right rectified image failed (%s), skipping loop",
		     triclopsErrorToString(triclops_->err));
    return;
  }
  err = triclopsGetImage(triclops_->context, TriImg_RECTIFIED, TriCam_LEFT, &image_left);
  if (err != TriclopsErrorOk) {
    logger->log_warn(name(), "Retrieving left rectified image failed (%s), skipping loop",
		     triclopsErrorToString(triclops_->err));
    return;
  }

  if (shm_img_right_rectified_->num_attached() > 1) {
    shm_img_right_rectified_->lock_for_write();
    memcpy(shm_img_right_rectified_->buffer(), image_right.data,
	   colorspace_buffer_size(MONO8, width_, height_));
    shm_img_right_rectified_->set_capture_time(&capture_ts);
    shm_img_right_rectified_->unlock();
  }
  if (shm_img_left_rectified_->num_attached() > 1) {
    shm_img_left_rectified_->lock_for_write();
    memcpy(shm_img_left_rectified_->buffer(), image_left.data,
	   colorspace_buffer_size(MONO8, width_, height_));
    shm_img_left_rectified_->set_capture_time(&capture_ts);
    shm_img_left_rectified_->unlock();
  }

  cv::Mat img_r(height_, width_, CV_8UC1, image_right.data);
  cv::Mat img_l(height_, width_, CV_8UC1, image_left.data);
  cv::Mat disparity(height_, width_, CV_16SC1); // CV_16SC1 CV_32FC1

  // Calculate disparity
  cv::StereoBM block_matcher(cv::StereoBM::BASIC_PRESET,
			     cfg_bm_num_disparities_, cfg_bm_sad_window_size_);;
  block_matcher.state->preFilterType     = cfg_bm_pre_filter_type_;
  block_matcher.state->preFilterSize     = cfg_bm_pre_filter_size_;
  block_matcher.state->preFilterCap      = cfg_bm_pre_filter_cap_;
  block_matcher.state->minDisparity 	 = cfg_bm_min_disparity_;
  block_matcher.state->textureThreshold  = cfg_bm_texture_threshold_;
  block_matcher.state->uniquenessRatio 	 = cfg_bm_uniqueness_ratio_;
  block_matcher.state->speckleWindowSize = cfg_bm_speckle_window_size_;
  block_matcher.state->speckleRange	 = cfg_bm_speckle_range_;
  block_matcher.state->trySmallerWindows = cfg_bm_try_smaller_widows_ ? 1 : 0;


  block_matcher(img_l, img_r, disparity);

  if (shm_img_right_prefiltered_->num_attached() > 1) {
    shm_img_right_prefiltered_->lock_for_write();
    memcpy(shm_img_right_prefiltered_->buffer(),
	   block_matcher.state->preFilteredImg0->data.ptr,
	   colorspace_buffer_size(MONO8, width_, height_));
    shm_img_right_prefiltered_->set_capture_time(&capture_ts);
    shm_img_right_prefiltered_->unlock();
  }
  if (shm_img_left_prefiltered_->num_attached() > 1) {
    shm_img_left_prefiltered_->lock_for_write();
    memcpy(shm_img_left_prefiltered_->buffer(),
	   block_matcher.state->preFilteredImg0->data.ptr,
	   colorspace_buffer_size(MONO8, width_, height_));
    shm_img_left_prefiltered_->set_capture_time(&capture_ts);
    shm_img_left_prefiltered_->unlock();
  }

  if (shm_img_disparity_->num_attached() > 1) {
    cv::Mat normalized_disparity(height_, width_, CV_16SC1);
    cv::normalize(disparity, normalized_disparity, 0, 256, cv::NORM_MINMAX);
    shm_img_disparity_->lock_for_write();
    unsigned char *buffer = shm_img_disparity_->buffer();
    for (unsigned int i = 0; i < width_ * height_; ++i) {
      buffer[i] = (unsigned char)((short int *)(normalized_disparity.data))[i];
    }
    shm_img_disparity_->set_capture_time(&capture_ts);
    shm_img_disparity_->unlock();
  }

  // Calculate 3D point cloud from data
  pcl::PointCloud<pcl::PointXYZ>    &pcl_xyz    = **pcl_xyz_;
  pcl::PointCloud<pcl::PointXYZRGB> &pcl_xyzrgb = **pcl_xyzrgb_;

  pcl_xyz.header.seq += 1;
  pcl_xyzrgb.header.seq += 1;
  pcl_utils::set_time(pcl_xyz_, capture_ts);
  pcl_utils::set_time(pcl_xyzrgb_, capture_ts);

  float focal_length;
  float baseline;
  float center_row, center_col;
  triclopsGetImageCenter(triclops_->context, &center_row, &center_col);
  triclopsGetFocalLength(triclops_->context, &focal_length);
  triclopsGetBaseline(triclops_->context, &baseline);

  //logger->log_debug(name(), "f: %f   B: %f  c: (%f,%f)",
  //		    focal_length, baseline, center_row, center_col);

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

  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < height_; ++h) {
    //const short* disp_cols = disparity.ptr<short>(h);
    for (unsigned int w = 0; w < width_; ++w, ++idx) {

      unsigned char *rgb = &buffer_rgb_right_[idx];

      pcl::PointXYZ &xyz = pcl_xyz.points[idx];
      pcl::PointXYZRGB &xyzrgb = pcl_xyzrgb.points[idx];

      //float u = (float)w - center_col;
      //float v = (float)h - center_row;

      double d = (double)disparity.at<short>(h, w);
      if (d <= FLT_EPSILON) {
	xyz.x = xyz.y = xyz.z = xyzrgb.x = xyzrgb.y = xyzrgb.z = bad_point;
	continue;
      }

      //float Z = baseline / d;//(float)disp_cols[w];
      
      float constant = focal_length * baseline;
      float tmpD = (constant / d);
      cv::Point2f sc =
	(cv::Point2f(center_col, center_row) - cv::Point2f (w, h)) * (tmpD / disparity.cols);
      double Z = tmpD;
      double X = sc.x;
      double Y = sc.y;

      xyz.x = xyzrgb.x = Z; //Z * focal_length;
      xyz.y = xyzrgb.y = X; //-u * Z;
      xyz.z = xyzrgb.z = Y; //-v * Z;

      //logger->log_debug(name(), "(X,Y,Z) = (%f,%f,%f)", xyz.x, xyz.y, xyz.z);

      xyzrgb.r = rgb[0];
      xyzrgb.g = rgb[1];
      xyzrgb.b = rgb[2];
    }
  }

#else
  #error No stereo matcher enabled
#endif

  bb2_->dispose_buffer();

}
