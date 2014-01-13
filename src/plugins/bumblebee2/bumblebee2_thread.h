
/***************************************************************************
 *  bumblebee2_thread.h - Acquire data from Bumblebee2 stereo camera
 *
 *  Created: Wed Jul 17 13:15:42 2013
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

#ifndef __PLUGINS_PERCEPTION_BUMBLEBEE2_BUMBLEBEE2_THREAD_H_
#define __PLUGINS_PERCEPTION_BUMBLEBEE2_BUMBLEBEE2_THREAD_H_

// must be first for reliable ROS detection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>
#include <aspect/pointcloud.h>

namespace fawkes {
  class SwitchInterface;
  class OpenCVStereoParamsInterface;
  class Time;
#ifdef USE_TIMETRACKER
  class TimeTracker;
#endif
}

namespace firevision {
  class Bumblebee2Camera;
  class SharedMemoryImageBuffer;
}

namespace cv {
  class Mat;
}

class TriclopsData;
class TriclopsColorImage;

class Bumblebee2Thread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect,
  public fawkes::PointCloudAspect
{
 public:
  Bumblebee2Thread();
  virtual ~Bumblebee2Thread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  void get_triclops_context_from_camera();
  void deinterlace_green(unsigned char* src, unsigned char* dest, 
			 unsigned int width, unsigned int height);
  void fill_xyz_xyzrgb(const short int *dispdata,
		       const TriclopsColorImage *img_right_rect_color,
		       pcl::PointCloud<pcl::PointXYZ> &pcl_xyz,
		       pcl::PointCloud<pcl::PointXYZRGB> &pcl_xyzrgb);
  void fill_xyzrgb(const short int *dispdata,
		   const TriclopsColorImage *img_rect_color,
		   pcl::PointCloud<pcl::PointXYZRGB> &pcl_xyzrgb);
  void fill_xyz(const short int *dispdata,
		pcl::PointCloud<pcl::PointXYZ> &pcl_xyz);


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  typedef enum {
    STEREO_MATCHER_TRICLOPS,
    STEREO_MATCHER_OPENCV
  } StereoMatcher;

  typedef enum {
    OPENCV_STEREO_BM,
    OPENCV_STEREO_SGBM
  } OpenCVStereoAlgorithm;

  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ> > pcl_xyz_;
  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZRGB> > pcl_xyzrgb_;

  fawkes::SwitchInterface *switch_if_;
  fawkes::OpenCVStereoParamsInterface *params_if_;

  firevision::Bumblebee2Camera *bb2_;
  TriclopsData *triclops_;

  unsigned int   width_;
  unsigned int   height_;
  float          baseline_;
  float          focal_length_;
  float          center_row_;
  float          center_col_;

  unsigned char *buffer_green_;
  unsigned char *buffer_rgb_;
  unsigned char *buffer_rgb_left_;
  unsigned char *buffer_rgb_right_;
  unsigned char *buffer_yuv_left_;
  unsigned char *buffer_yuv_right_;
  unsigned char *buffer_rgb_planar_left_;
  unsigned char *buffer_rgb_planar_right_;

  firevision::SharedMemoryImageBuffer *shm_img_rgb_right_;
  firevision::SharedMemoryImageBuffer *shm_img_rgb_left_;
  firevision::SharedMemoryImageBuffer *shm_img_yuv_right_;
  firevision::SharedMemoryImageBuffer *shm_img_yuv_left_;
  firevision::SharedMemoryImageBuffer *shm_img_rectified_right_;
  firevision::SharedMemoryImageBuffer *shm_img_rectified_left_;
  firevision::SharedMemoryImageBuffer *shm_img_prefiltered_right_;
  firevision::SharedMemoryImageBuffer *shm_img_prefiltered_left_;
  firevision::SharedMemoryImageBuffer *shm_img_rgb_rect_left_;
  firevision::SharedMemoryImageBuffer *shm_img_rgb_rect_right_;
  firevision::SharedMemoryImageBuffer *shm_img_disparity_;

  std::string   cfg_base_frame_;
  std::string   cfg_frames_prefix_;
  float         cfg_frames_interval_;
  StereoMatcher cfg_stereo_matcher_;

  // OpenCV-specific settings
  OpenCVStereoAlgorithm cfg_opencv_stereo_algorithm_;
  int           cfg_bm_pre_filter_type_;
  unsigned int  cfg_bm_pre_filter_size_;
  unsigned int  cfg_bm_pre_filter_cap_;
  unsigned int  cfg_bm_sad_window_size_;
  int           cfg_bm_min_disparity_;
  unsigned int  cfg_bm_num_disparities_;
  unsigned int  cfg_bm_texture_threshold_;
  unsigned int  cfg_bm_uniqueness_ratio_;
  unsigned int  cfg_bm_speckle_window_size_;
  unsigned int  cfg_bm_speckle_range_;
  bool          cfg_bm_try_smaller_windows_;

  bool          cfg_sgbm_p1_auto_;
  bool          cfg_sgbm_p2_auto_;
  int           cfg_sgbm_p1_;
  int           cfg_sgbm_p2_;
  int           cfg_sgbm_disp_12_max_diff_;

  float         disparity_scale_factor_;

  cv::Mat *cv_disparity_;

  fawkes::Time                 *tf_last_publish_;
  fawkes::tf::StampedTransform *tf_left_;
  fawkes::tf::StampedTransform *tf_right_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker  *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_full_loop_;
  unsigned int ttc_transforms_;
  unsigned int ttc_msgproc_;
  unsigned int ttc_capture_;
  unsigned int ttc_preprocess_;
  unsigned int ttc_rectify_;
  unsigned int ttc_stereo_match_;
  unsigned int ttc_pcl_xyzrgb_;
  unsigned int ttc_pcl_xyz_;
#endif
};

#endif
