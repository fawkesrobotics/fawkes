
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

#ifndef _PLUGINS_LASER_LINES_LASER_LINES_THREAD_H_
#define _PLUGINS_LASER_LINES_LASER_LINES_THREAD_H_

// must be first for reliable ROS detection
#include "line_info.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/pointcloud.h>
#include <aspect/tf.h>
#include <config/change_handler.h>
#include <core/threading/thread.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/StdVector>

#ifdef HAVE_VISUAL_DEBUGGING
#	include <plugins/ros/aspect/ros.h>
#	include <visualization_msgs/MarkerArray.h>

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
} // namespace fawkes

class LaserLinesThread : public fawkes::Thread,
                         public fawkes::ClockAspect,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::BlockedTimingAspect,
                         public fawkes::ConfigurationChangeHandler,
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
	typedef pcl::PointXYZ              PointType;
	typedef pcl::PointCloud<PointType> Cloud;
	typedef Cloud::Ptr                 CloudPtr;
	typedef Cloud::ConstPtr            CloudConstPtr;

	typedef pcl::PointXYZRGB                ColorPointType;
	typedef pcl::PointCloud<ColorPointType> ColorCloud;
	typedef ColorCloud::Ptr                 ColorCloudPtr;
	typedef ColorCloud::ConstPtr            ColorCloudConstPtr;

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

protected:
	virtual void read_config();
	virtual void
	config_tag_changed(const char *)
	{
	}
	virtual void
	config_comment_changed(const fawkes::Configuration::ValueIterator *)
	{
	}
	virtual void
	config_value_changed(const fawkes::Configuration::ValueIterator *v)
	{
		read_config();
	}
	virtual void
	config_value_erased(const char *path)
	{
		read_config();
	}

private:
	void update_lines(std::vector<LineInfo> &);
	void publish_known_lines();

	void set_interface(unsigned int                idx,
	                   fawkes::LaserLineInterface *iface,
	                   bool                        moving_average,
	                   bool                        transformed,
	                   const TrackedLineInfo      &tinfo,
	                   const std::string          &frame_id = "");

	void set_empty_interface(fawkes::LaserLineInterface *iface) const;

#ifdef HAVE_VISUAL_DEBUGGING
	void publish_visualization(const std::vector<TrackedLineInfo> &linfos,
	                           const std::string                  &marker_namespace,
	                           const std::string                  &avg_marker_namespace);

	void publish_visualization_add_line(visualization_msgs::MarkerArray &m,
	                                    unsigned int                    &idnum,
	                                    const LineInfo                  &info,
	                                    const size_t                     i,
	                                    const std::string               &marker_namespace,
	                                    const std::string               &name_suffix = "");
#endif

private:
	fawkes::RefPtr<const pcl::PointCloud<PointType>> finput_;
	fawkes::RefPtr<pcl::PointCloud<ColorPointType>>  flines_;
	CloudConstPtr                                    input_;
	pcl::PointCloud<ColorPointType>::Ptr             lines_;

	std::vector<fawkes::LaserLineInterface *> line_ifs_;
	std::vector<fawkes::LaserLineInterface *> line_avg_ifs_;
	std::vector<fawkes::LaserLineInterface *> line_transformed_ifs_;
	std::vector<TrackedLineInfo>              known_lines_;

	fawkes::SwitchInterface *switch_if_;

	typedef enum { SELECT_MIN_ANGLE, SELECT_MIN_DIST } selection_mode_t;

	unsigned int cfg_segm_max_iterations_;
	float        cfg_segm_distance_threshold_;
	float        cfg_segm_sample_max_dist_;
	float        cfg_min_length_;
	float        cfg_max_length_;
	unsigned int cfg_segm_min_inliers_;
	std::string  cfg_input_pcl_;
	unsigned int cfg_max_num_lines_;
	float        cfg_switch_tolerance_;
	float        cfg_cluster_tolerance_;
	float        cfg_cluster_quota_;
	float        cfg_min_dist_;
	float        cfg_max_dist_;
	bool         cfg_moving_avg_enabled_;
	unsigned int cfg_moving_avg_window_size_;
	bool         cfg_transform_to_frame_enabled_;
	std::string  cfg_transform_to_frame_id_;
	std::string  cfg_tracking_frame_id_;

	unsigned int loop_count_;

#ifdef USE_TIMETRACKER
	fawkes::TimeTracker *tt_;
	unsigned int         tt_loopcount_;
	unsigned int         ttc_full_loop_;
	unsigned int         ttc_msg_proc_;
	unsigned int         ttc_extract_lines_;
	unsigned int         ttc_clustering_;
#endif

#ifdef HAVE_VISUAL_DEBUGGING
	ros::Publisher *vispub_;
	size_t          last_id_num_;
#endif
};

#endif
