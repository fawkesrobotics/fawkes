
/***************************************************************************
 *  realsense_thread.h - realsense
 *
 *  Plugin created: Mon Jun 13 17:09:44 2016

 *  Copyright  2016  Johannes Rothe
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

#ifndef __PLUGINS_REALSENSETHREAD_H_
#define __PLUGINS_REALSENSETHREAD_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/pointcloud.h>
#include <aspect/clock.h>

#ifdef HAVE_REALSENSE1
#include <librealsense1/rs.hpp>
#else
#include <librealsense/rs.hpp>
#endif

#include <string>

namespace fawkes {
  class SwitchInterface;
}

class RealsenseThread 
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::PointCloudAspect,
  public fawkes::ClockAspect
{

 public:
  RealsenseThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  bool connect_and_start_camera();
  rs_device* get_camera();
  void enable_depth_stream();
  void log_error();
  void log_depths(const uint16_t * image);
  void fill_pointcloud();
  void stop_camera();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() { Thread::run(); }

 protected:
  bool read_switch();

 private:
  fawkes::SwitchInterface *switch_if_;
  bool cfg_use_switch_;

  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> Cloud;

  typedef Cloud::Ptr CloudPtr;
  typedef Cloud::ConstPtr CloudConstPtr;

  fawkes::RefPtr<Cloud> realsense_depth_refptr_;
  CloudPtr realsense_depth_;

  rs_error *rs_error_ = 0;
  rs_context *rs_context_;
  rs_device *rs_device_;
  rs_intrinsics z_intrinsic_;
  rs_stream rs_stream_type_;
  int num_of_cameras_;
  float camera_scale_;
  std::string frame_id_;
  std::string pcl_id_;
  bool camera_started_ = false;
  int laser_power_;

};


#endif
