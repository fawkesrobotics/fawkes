
/***************************************************************************
 *  laser-cluster-thread.h - Thread to detect a cluster in 2D laser data
 *
 *  Created: Sun Apr 21 01:17:09 2013
 *  Copyright  2011-2015  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_LASER_CLUSTER_LASER_CLUSTER_THREAD_H_
#define _PLUGINS_LASER_CLUSTER_LASER_CLUSTER_THREAD_H_

// must be first for reliable ROS detection
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/pointcloud.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/StdVector>

namespace fawkes {
class Position3DInterface;
class SwitchInterface;
#ifdef USE_TIMETRACKER
class TimeTracker;
#endif
class LaserClusterInterface;
} // namespace fawkes

class LaserClusterThread : public fawkes::Thread,
                           public fawkes::ClockAspect,
                           public fawkes::LoggingAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::BlackBoardAspect,
                           public fawkes::BlockedTimingAspect,
                           public fawkes::TransformAspect,
                           public fawkes::PointCloudAspect
{
public:
	LaserClusterThread(std::string &cfg_name, std::string &cfg_prefix);
	virtual ~LaserClusterThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	typedef pcl::PointXYZ              PointType;
	typedef pcl::PointCloud<PointType> Cloud;
	typedef Cloud::Ptr                 CloudPtr;
	typedef Cloud::ConstPtr            CloudConstPtr;

	typedef pcl::PointXYZRGB                ColorPointType;
	typedef pcl::PointCloud<ColorPointType> ColorCloud;
	typedef ColorCloud::Ptr                 ColorCloudPtr;
	typedef ColorCloud::ConstPtr            ColorCloudConstPtr;

	typedef pcl::PointXYZL                  LabelPointType;
	typedef pcl::PointCloud<LabelPointType> LabelCloud;
	typedef LabelCloud::Ptr                 LabelCloudPtr;
	typedef LabelCloud::ConstPtr            LabelCloudConstPtr;

private:
	void set_position(fawkes::Position3DInterface *iface,
	                  bool                         is_visible,
	                  const Eigen::Vector4f       &centroid = Eigen::Vector4f(0, 0, 0, 0),
	                  const Eigen::Quaternionf    &rotation = Eigen::Quaternionf(1, 0, 0, 0));

	float calc_line_length(CloudPtr                    cloud,
	                       pcl::PointIndices::Ptr      inliers,
	                       pcl::ModelCoefficients::Ptr coeff);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::RefPtr<const pcl::PointCloud<PointType>> finput_;
	fawkes::RefPtr<pcl::PointCloud<ColorPointType>>  fclusters_;
	fawkes::RefPtr<pcl::PointCloud<LabelPointType>>  fclusters_labeled_;
	CloudConstPtr                                    input_;
	pcl::PointCloud<ColorPointType>::Ptr             clusters_;
	pcl::PointCloud<LabelPointType>::Ptr             clusters_labeled_;

	pcl::SACSegmentation<PointType> seg_;

	std::vector<fawkes::Position3DInterface *> cluster_pos_ifs_;

	fawkes::SwitchInterface       *switch_if_;
	fawkes::LaserClusterInterface *config_if_;

	typedef enum { SELECT_MIN_ANGLE, SELECT_MIN_DIST } selection_mode_t;

	std::string cfg_name_;
	std::string cfg_prefix_;

	bool             cfg_line_removal_;
	unsigned int     cfg_segm_max_iterations_;
	float            cfg_segm_distance_threshold_;
	unsigned int     cfg_segm_min_inliers_;
	float            cfg_segm_sample_max_dist_;
	float            cfg_line_min_length_;
	float            cfg_cluster_tolerance_;
	unsigned int     cfg_cluster_min_size_;
	unsigned int     cfg_cluster_max_size_;
	std::string      cfg_input_pcl_;
	std::string      cfg_result_frame_;
	float            cfg_bbox_min_x_;
	float            cfg_bbox_max_x_;
	float            cfg_bbox_min_y_;
	float            cfg_bbox_max_y_;
	bool             cfg_use_bbox_;
	float            cfg_switch_tolerance_;
	float            cfg_offset_x_;
	float            cfg_offset_y_;
	float            cfg_offset_z_;
	selection_mode_t cfg_selection_mode_;
	unsigned int     cfg_max_num_clusters_;

	std::string output_cluster_name_;
	std::string output_cluster_labeled_name_;

	float current_max_x_;

	unsigned int loop_count_;

	class ClusterInfo
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		double          angle;
		double          dist;
		unsigned int    index;
		Eigen::Vector4f centroid;
	};

#ifdef USE_TIMETRACKER
	fawkes::TimeTracker *tt_;
	unsigned int         tt_loopcount_;
	unsigned int         ttc_full_loop_;
	unsigned int         ttc_msg_proc_;
	unsigned int         ttc_extract_lines_;
	unsigned int         ttc_clustering_;
#endif
};

#endif
