
/***************************************************************************
 *  laser-lines-thread.h - Thread to detect a lines in 2D laser data
 *
 *  Created: Fri May 23 18:10:54 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_LINES_LASER_LINES_THREAD_H_
#define __PLUGINS_LASER_LINES_LASER_LINES_THREAD_H_

// must be first for reliable ROS detection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "line_info.h"

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/tf.h>
#include <aspect/pointcloud.h>

#include <Eigen/StdVector>
#include <pcl/ModelCoefficients.h>

#ifdef HAVE_VISUAL_DEBUGGING
#  include <plugins/ros/aspect/ros.h>

namespace ros {
  class Publisher;
}
#endif


namespace fawkes {
  class Position3DInterface;
  class SwitchInterface;
#ifdef USE_TIMETRACKER
  class TimeTracker;
#endif
  class LaserLineInterface;
}

class LaserLinesThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::TransformAspect,
#ifdef HAVE_VISUAL_DEBUGGING
  public fawkes::ROSAspect,
#endif
  public fawkes::PointCloudAspect
{
 public:
  LaserLinesThread();
  virtual ~LaserLinesThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> Cloud;
  typedef Cloud::Ptr CloudPtr;
  typedef Cloud::ConstPtr CloudConstPtr;

  typedef pcl::PointXYZRGB ColorPointType;
  typedef pcl::PointCloud<ColorPointType> ColorCloud;
  typedef ColorCloud::Ptr ColorCloudPtr;
  typedef ColorCloud::ConstPtr ColorCloudConstPtr;

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:

  void set_line(fawkes::LaserLineInterface *iface,
		bool is_visible,
		const std::string &frame_id = "",
		const LineInfo &info = LineInfo());


#ifdef HAVE_VISUAL_DEBUGGING
  void publish_visualization(const std::vector<LineInfo> &linfos);
#endif

 private:
  fawkes::RefPtr<const pcl::PointCloud<PointType> > finput_;
  fawkes::RefPtr<pcl::PointCloud<ColorPointType> > flines_;
  CloudConstPtr input_;
  pcl::PointCloud<ColorPointType>::Ptr lines_;

  std::vector<fawkes::LaserLineInterface *> line_ifs_;

  fawkes::SwitchInterface *switch_if_;

  typedef enum  {
    SELECT_MIN_ANGLE,
    SELECT_MIN_DIST
  } selection_mode_t;

  unsigned int cfg_segm_max_iterations_;
  float        cfg_segm_distance_threshold_;
  float        cfg_segm_sample_max_dist_;
  float        cfg_min_length_;
  float        cfg_max_length_;
  unsigned int cfg_segm_min_inliers_;
  std::string  cfg_input_pcl_;
  std::string  cfg_result_frame_;
  unsigned int cfg_max_num_lines_;
  float        cfg_switch_tolerance_;
  float        cfg_cluster_tolerance_;
  float        cfg_cluster_quota_;
  float        cfg_min_dist_;
  float        cfg_max_dist_;

  unsigned int loop_count_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker  *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_full_loop_;
  unsigned int ttc_msg_proc_;
  unsigned int ttc_extract_lines_;
  unsigned int ttc_clustering_;
#endif

#ifdef HAVE_VISUAL_DEBUGGING
  ros::Publisher *vispub_;
  size_t last_id_num_;
#endif

};

#endif
