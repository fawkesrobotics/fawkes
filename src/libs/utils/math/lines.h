
/***************************************************************************
 *  lines.h - functions to operate on lines
 *
 *  Created: Tue Apr 07 21:42:34 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __UTILS_MATH_LINES_H_
#define __UTILS_MATH_LINES_H_

#include <Eigen/Geometry>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Check if two line segments intersect.
 * Line segments only intersect if the intersection point of the lines
 * lies within both segment boundaries.
 * @param l1_from line segment 1 first point
 * @param l1_to line segment 1 second point
 * @param l2_from line segment 2 first point
 * @param l2_to line segment 2 second point
 * @return true if the lines intersect, false otherwise
 */
bool
line_segm_intersect(const Eigen::Vector2f l1_from, const Eigen::Vector2f l1_to,
		    const Eigen::Vector2f l2_from, const Eigen::Vector2f l2_to)
{
  const Eigen::ParametrizedLine<float, 2>
    edge_seg(Eigen::ParametrizedLine<float, 2>::Through(l1_from, l1_to));

  const Eigen::ParametrizedLine<float, 2>
    line_seg(Eigen::ParametrizedLine<float, 2>::Through(l2_from, l2_to));

  float k = edge_seg.direction().dot(line_seg.direction());
  if (std::abs(k - 1.0) < std::numeric_limits<double>::epsilon()) {
    // lines are collinear, check overlap

    // lines are actually parallel with a non-zero distance
    if (edge_seg.distance(l2_from) > std::numeric_limits<float>::epsilon())  return false;

    // Check if l2_from or l2_to is in the edge line segment
    Eigen::Vector2f dir = l1_to - l1_from;
    float dir_sn = dir.squaredNorm();
    float k = dir.dot(l2_from - l1_from) / dir_sn;
    if (k >= 0. && k <= 1.)  return true;

    k = dir.dot(l2_to - l1_from) / dir_sn;
    if (k >= 0. && k <= 1.)  return true;

    // Check if the edge end points are in the l2_from--l2_to line segment
    dir = l2_to - l2_from;
    dir_sn = dir.squaredNorm();
    k = dir.dot(l1_from - l2_from) / dir_sn;
    if (k >= 0. && k <= 1.)  return true;

    k = dir.dot(l1_to - l2_from) / dir_sn;
    if (k >= 0. && k <= 1.)  return true;

    // collinear, but not overlapping
    return false;

  } else {
    const float ipar =
      edge_seg.intersection(Eigen::Hyperplane<float, 2>(line_seg));

    if (std::isfinite(ipar)) {
#if EIGEN_VERSION_AT_LEAST(3,2,0)
      const Eigen::Vector2f ip(edge_seg.pointAt(ipar));
#else
      const Eigen::Vector2f ip(edge_seg.origin() + (edge_seg.direction()*ipar));
#endif
      // check if the intersection point is on the line segments
      Eigen::Vector2f dir_edge = l1_to - l1_from;
      Eigen::Vector2f dir_line = l2_to - l2_from;
      float k_edge = dir_edge.dot(ip - l1_from) / dir_edge.squaredNorm();
      float k_line = dir_line.dot(ip - l2_from) / dir_line.squaredNorm();
      return (k_edge >= 0. && k_edge <= 1. && k_line >= 0. && k_line <= 1.);

    } else {
      return false;
    }
  }
}

/** Get line segment intersection point.
 * Line segments only intersect if the intersection point of the lines
 * lies within both segment boundaries.
 * @param l1_from line segment 1 first point
 * @param l1_to line segment 1 second point
 * @param l2_from line segment 2 first point
 * @param l2_to line segment 2 second point
 * @return point which is either the intersection point, or a point of NaNs if
 * no intersection point exists.
 */
Eigen::Vector2f
line_segm_intersection(const Eigen::Vector2f l1_from, const Eigen::Vector2f l1_to,
		       const Eigen::Vector2f l2_from, const Eigen::Vector2f l2_to)
{
  const Eigen::ParametrizedLine<float, 2>
    edge_seg(Eigen::ParametrizedLine<float, 2>::Through(l1_from, l1_to));

  const Eigen::ParametrizedLine<float, 2>
    line_seg(Eigen::ParametrizedLine<float, 2>::Through(l2_from, l2_to));

  float k = edge_seg.direction().dot(line_seg.direction());
  if (std::abs(k - 1.0) < std::numeric_limits<double>::epsilon()) {
    // lines are collinear, check overlap

    // lines are actually parallel with a non-zero distance
    if (edge_seg.distance(l2_from) > std::numeric_limits<float>::epsilon())
      return Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(),
			     std::numeric_limits<float>::quiet_NaN());


    // Check if l2_from or l2_to is in the edge line segment
    Eigen::Vector2f dir = l1_to - l1_from;
    float dir_sn = dir.squaredNorm();
    float k = dir.dot(l2_from - l1_from) / dir_sn;
    if (k >= 0. && k <= 1.)  return l2_from;

    k = dir.dot(l2_to - l1_from) / dir_sn;
    if (k >= 0. && k <= 1.)  return l2_to;

    // Check if the edge end points are in the l2_from--l2_to line segment
    dir = l2_to - l2_from;
    dir_sn = dir.squaredNorm();
    k = dir.dot(l1_from - l2_from) / dir_sn;
    if (k >= 0. && k <= 1.)  return l1_from;

    k = dir.dot(l1_to - l2_from) / dir_sn;
    if (k >= 0. && k <= 1.)  return l1_to;

    // collinear, but not overlapping
    return Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(),
			   std::numeric_limits<float>::quiet_NaN());

  } else {
    const float ipar =
      edge_seg.intersection(Eigen::Hyperplane<float, 2>(line_seg));

    if (std::isfinite(ipar)) {
#if EIGEN_VERSION_AT_LEAST(3,2,0)
      const Eigen::Vector2f ip(edge_seg.pointAt(ipar));
#else
      const Eigen::Vector2f ip(edge_seg.origin() + (edge_seg.direction()*ipar));
#endif
      // check if the intersection point is on the line segments
      Eigen::Vector2f dir_edge = l1_to - l1_from;
      Eigen::Vector2f dir_line = l2_to - l2_from;
      float k_edge = dir_edge.dot(ip - l1_from) / dir_edge.squaredNorm();
      float k_line = dir_line.dot(ip - l2_from) / dir_line.squaredNorm();
      if (k_edge >= 0. && k_edge <= 1. && k_line >= 0. && k_line <= 1.) {
	return ip;
      } else {
	return Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(),
			       std::numeric_limits<float>::quiet_NaN());
      }


    } else {
      return Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(),
			     std::numeric_limits<float>::quiet_NaN());
    }
  }

}


} // end namespace fawkes

#endif
