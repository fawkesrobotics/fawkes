
/***************************************************************************
 *  line_info.h - line info container
 *
 *  Created: Tue Mar 17 11:13:24 2015 (re-factoring)
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

#ifndef _PLUGINS_LASER_LINES_LINE_INFO_H_
#define _PLUGINS_LASER_LINES_LINE_INFO_H_

#include <logging/logger.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transformer.h>
#include <tf/types.h>

#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>
#include <memory>

/** Line information container.
 * All points and angles are in the sensor reference frame
 * from which the lines were extracted.
 */
class LineInfo
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	float bearing; ///< bearing to point on line
	float length;  ///< length of the detecte line segment

	Eigen::Vector3f point_on_line;  ///< point on line vector
	Eigen::Vector3f line_direction; ///< line direction vector

	Eigen::Vector3f base_point; ///< optimized closest point on line

	Eigen::Vector3f end_point_1; ///< line segment end point
	Eigen::Vector3f end_point_2; ///< line segment end point

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; ///< point cloud consisting only of
	                                           ///< points account to this line
};

class TrackedLineInfo
{
public:
	int interface_idx; ///< id of the interface, this line is written to, -1 when not yet assigned
	int visibility_history; ///< visibility history of this line, negative for "no sighting"
	LineInfo raw;           ///< the latest geometry of this line, i.e. unfiltered
	LineInfo smooth;        ///< moving-average geometry of this line (cf. length of history buffer)
	LineInfo
	  transformed; ///< the latest geometry of this line, i.e. unfiltered - in fixed frame (i.e. map)
	fawkes::tf::Stamped<fawkes::tf::Point>
	  base_point_odom; ///< last reference point (in odom frame) for line tracking
	fawkes::tf::Stamped<fawkes::tf::Point>
	  base_point_fixed; ///< last reference point (in odom frame) for line tracking
	fawkes::tf::Transformer
             *transformer;    ///< Transformer used to transform from input_frame_id_to odom
	std::string input_frame_id; ///< Input frame ID of raw line infos (base_laser usually)
	std::string fixed_frame_id; ///< Input frame ID of raw line infos (base_laser usually)
	bool        transform_to_fixed_frame; ///< Whether to transform the line to the fixed_frame
	std::string
	  tracking_frame_id; ///< Track lines relative to this frame (e.g. odom helps compensate movement)
	float cfg_switch_tolerance; ///< Configured line jitter threshold
	boost::circular_buffer<LineInfo>
	  history; ///< history of raw line geometries for computing moving average
	float
	  bearing_center; ///< Bearing towards line center, used to select lines "in front of us" when there
	fawkes::Logger *logger;      ///< Logger pointer of the calling class
	std::string     plugin_name; ///< Plugin name of the calling class

	TrackedLineInfo(fawkes::tf::Transformer *tfer,
	                const std::string       &input_frame_id,
	                const std::string       &tracking_frame_id,
	                const std::string       &fixed_frame_id,
	                bool                     transform_to_fixed_frame,
	                float                    cfg_switch_tolerance,
	                unsigned int             cfg_moving_avg_len,
	                fawkes::Logger          *logger,
	                const std::string       &plugin_name);

	btScalar distance(const LineInfo &linfo) const;
	void     update(LineInfo &new_linfo);
	void     not_visible_update();
	bool     transform_point_to_frame(const Eigen::Vector3f &in_point,
	                                  Eigen::Vector3f       &out_point,
	                                  const std::string     &from_frame,
	                                  const std::string     &to_frame);
};

#endif
