
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

#ifndef __PLUGINS_LASER_LINES_LINE_INFO_H_
#define __PLUGINS_LASER_LINES_LINE_INFO_H_

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** Line information container.
 * All points and angles are in the sensor reference frame
 * from which the lines were extracted.
 */
class LineInfo {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  float bearing;	///< bearing to point on line
  float length;		///< length of the detecte line segment

  Eigen::Vector3f point_on_line;	///< point on line vector
  Eigen::Vector3f line_direction;	///< line direction vector

  Eigen::Vector3f base_point;		///< optimized closest point on line

  Eigen::Vector3f end_point_1;		///< line segment end point
  Eigen::Vector3f end_point_2;		///< line segment end point

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;	///< point cloud consisting only of
  						///< points account to this line
};

#endif
