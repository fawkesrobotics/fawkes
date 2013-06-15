
/***************************************************************************
 *  laser-cluster-thread.h - Thread to detect a cluster in 2D laser data
 *
 *  Created: Sun Apr 21 01:17:09 2013
 *  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_CLUSTER_LASER_CLUSTER_THREAD_H_
#define __PLUGINS_LASER_CLUSTER_LASER_CLUSTER_THREAD_H_

// must be first for reliable ROS detection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/tf.h>
#include <aspect/pointcloud.h>

#include <Eigen/StdVector>
#include <pcl/segmentation/sac_segmentation.h>

namespace fawkes {
  class Position3DInterface;
  class SwitchInterface;
#ifdef USE_TIMETRACKER
  class TimeTracker;
#endif
  class LaserClusterInterface;
}

class LaserClusterThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::TransformAspect,
  public fawkes::PointCloudAspect
{
 public:
  LaserClusterThread();
  virtual ~LaserClusterThread();

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

 private:
  void set_position(fawkes::Position3DInterface *iface,
                    bool is_visible,
		    const Eigen::Vector4f &centroid = Eigen::Vector4f(0, 0, 0, 0),
                    const Eigen::Quaternionf &rotation = Eigen::Quaternionf(1, 0, 0, 0));

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::RefPtr<const pcl::PointCloud<PointType> > finput_;
  fawkes::RefPtr<pcl::PointCloud<ColorPointType> > fclusters_;
  CloudConstPtr input_;
  pcl::PointCloud<ColorPointType>::Ptr clusters_;

  pcl::SACSegmentation<PointType> seg_;

  fawkes::Position3DInterface *cluster_pos_if_;

  fawkes::SwitchInterface *switch_if_;
  fawkes::LaserClusterInterface *config_if_;

  bool         cfg_line_removal_;
  float        cfg_depth_filter_min_x_;
  float        cfg_depth_filter_max_x_;
  unsigned int cfg_segm_max_iterations_;
  float        cfg_segm_distance_threshold_;
  unsigned int cfg_segm_min_inliers_;
  float        cfg_cluster_tolerance_;
  unsigned int cfg_cluster_min_size_;
  unsigned int cfg_cluster_max_size_;
  std::string  cfg_input_pcl_;
  std::string  cfg_result_frame_;
  float        cfg_cluster_min_x_;
  float        cfg_cluster_max_x_;
  float        cfg_cluster_min_y_;
  float        cfg_cluster_max_y_;
  float        cfg_cluster_switch_tolerance_;
  float        cfg_offset_x_;

  float        current_max_x_;

  unsigned int loop_count_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker  *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_full_loop_;
  unsigned int ttc_msg_proc_;
  unsigned int ttc_extract_lines_;
  unsigned int ttc_clustering_;
#endif

};

#endif
