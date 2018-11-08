
/***************************************************************************
 *  pointcloud_thread.h - OpenNI point cloud provider thread
 *
 *  Created: Fri Mar 25 23:48:05 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef _PLUGINS_OPENNI_POINTCLOUD_THREAD_H_
#define _PLUGINS_OPENNI_POINTCLOUD_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#ifdef HAVE_PCL
#  include <pcl/point_cloud.h>
#  include <pcl/point_types.h>
#  include <fvutils/adapters/pcl.h>
#  include <aspect/pointcloud.h>
#endif
#include <plugins/openni/aspect/openni.h>

// OpenNI relies on GNU extension to detect Linux
#if defined(__linux__) && not defined(linux)
#  define linux true
#endif
#if defined(__i386__) && not defined(i386)
#  define i386 true
#endif
#include <XnCppWrapper.h>

#include <map>

namespace fawkes {
  class Time;
}

namespace firevision {
  class SharedMemoryImageBuffer;
}

class OpenNiImageThread;

class OpenNiPointCloudThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
#ifdef HAVE_PCL
  public fawkes::PointCloudAspect,
#endif
  public fawkes::OpenNiAspect
{
 public:
  OpenNiPointCloudThread(OpenNiImageThread *img_thread);
  virtual ~OpenNiPointCloudThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void fill_xyz_no_pcl(fawkes::Time &ts, const XnDepthPixel * const data);
  void fill_xyzrgb_no_pcl(fawkes::Time &ts, const XnDepthPixel * const data);
  void fill_xyz_xyzrgb_no_pcl(fawkes::Time &ts, const XnDepthPixel * const data);
  void fill_rgb_no_pcl();

#ifdef HAVE_PCL
  void fill_xyz(fawkes::Time &ts, const XnDepthPixel * const depth_data);
  void fill_xyzrgb(fawkes::Time &ts, const XnDepthPixel * const depth_data);
  void fill_xyz_xyzrgb(fawkes::Time &ts, const XnDepthPixel * const depth_data);
  void fill_rgb(pcl::PointCloud<pcl::PointXYZRGB> &pcl_rgb);
#endif

 private:
  OpenNiImageThread *img_thread_;

  xn::DepthGenerator  *depth_gen_;
  xn::ImageGenerator  *image_gen_;
  xn::DepthMetaData   *depth_md_;

  bool         cfg_register_depth_image_;

  firevision::SharedMemoryImageBuffer *pcl_xyz_buf_;
  firevision::SharedMemoryImageBuffer *pcl_xyzrgb_buf_;

  firevision::SharedMemoryImageBuffer *image_rgb_buf_;

  float        focal_length_;
  float        foc_const_; // focal length constant used in conversion
  float        center_x_;
  float        center_y_;
  unsigned int width_;
  unsigned int height_;

  XnUInt64     no_sample_value_;
  XnUInt64     shadow_value_;

  fawkes::Time *capture_start_;

  std::string cfg_frame_depth_;
  std::string cfg_frame_image_;

#ifdef HAVE_PCL
  bool         cfg_generate_pcl_;

  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZ> >    pcl_xyz_;
  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZRGB> > pcl_xyzrgb_;
#endif
};

#endif
