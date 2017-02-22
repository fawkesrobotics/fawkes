
/***************************************************************************
 *  line_info.cpp - line info container
 *
 *  Created: Tue Mar 17 11:13:24 2015 (re-factoring)
 *  Copyright  2011-2015  Tim Niemueller [www.niemueller.de]
 *                  2016  Victor Mataré
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

#include "line_info.h"

using namespace std;

/** @class TrackedLineInfo "line_info.h"
 * Container for a line with tracking and smoothing info.
 */

/** Constructor.
 * @param tfer tf transformer
 * @param input_frame_id frame id of incoming data
 * @param tracking_frame_id fixed frame in which to perform tracking
 * @param cfg_switch_tolerance tolerance in m for when to assume a line ID switch
 * @param cfg_moving_avg_len length of buffer for moving average
 * @param logger logger for informational messages
 * @param plugin_name component for informational messages
 */
TrackedLineInfo::TrackedLineInfo(
    fawkes::tf::Transformer *tfer,
    const string &input_frame_id,
    const string &tracking_frame_id,
    float cfg_switch_tolerance,
    unsigned int cfg_moving_avg_len,
    fawkes::Logger *logger,
    string plugin_name)
: interface_idx(-1),
  visibility_history(0),
  transformer(tfer),
  input_frame_id(input_frame_id),
  tracking_frame_id(tracking_frame_id),
  cfg_switch_tolerance(cfg_switch_tolerance),
  history(cfg_moving_avg_len),
  bearing_center(0),
  logger(logger),
  plugin_name(plugin_name)
{}


/** Compute this line's distance from line info
 * @param linfo line info
 * @return the scalar distance between the two base points in meters.
 */
btScalar TrackedLineInfo::distance(const LineInfo &linfo) const
{
  fawkes::tf::Stamped<fawkes::tf::Point> bp_new(
	  fawkes::tf::Point(
	      linfo.base_point[0], linfo.base_point[1], linfo.base_point[2]
	  ), fawkes::Time(0,0), input_frame_id);
  fawkes::tf::Stamped<fawkes::tf::Point> bp_odom_new;
  try {
    transformer->transform_point(tracking_frame_id, bp_new, bp_odom_new);
  } catch (fawkes::tf::TransformException &e) {
    // Continue without tf, track in input frame instead. Warning follows on update() call.
    bp_odom_new = bp_new;
  }

  return (bp_odom_new - this->base_point_odom).length();
}

/**
 * Update this currently not visible line, make the visibility history (more) negative
 * and invalidate the data.
 */
void TrackedLineInfo::not_visible_update() {
	if (visibility_history >= 0)
	  visibility_history = -1;
	else
	  visibility_history -= 1;

	this->raw.cloud.reset();
	this->smooth.cloud.reset();
}

/** Update this line.
 * @param linfo new info to consume
 * This also updates moving averages for all fields.
 */
void TrackedLineInfo::update(LineInfo &linfo)
{
  if (visibility_history <= 0)
    visibility_history = 1;
  else
    visibility_history += 1;

  this->raw = linfo;
  fawkes::tf::Stamped<fawkes::tf::Point> bp_new(
	  fawkes::tf::Point(
	      linfo.base_point[0], linfo.base_point[1], linfo.base_point[2]
	  ), fawkes::Time(0,0), input_frame_id);
  try {
    transformer->transform_point(tracking_frame_id, bp_new, this->base_point_odom);
  } catch (fawkes::tf::TransformException &e) {
    logger->log_warn(plugin_name.c_str(), "Can't transform to %s. Attempting to track in %s.",
										 tracking_frame_id.c_str(), input_frame_id.c_str());
    this->base_point_odom = bp_new;
  }
  this->history.push_back(linfo);

  Eigen::Vector3f base_point_sum(0,0,0), end_point_1_sum(0,0,0),
	  end_point_2_sum(0,0,0), line_direction_sum(0,0,0), point_on_line_sum(0,0,0);
  float length_sum(0);
  for (LineInfo &l : this->history) {
	base_point_sum += l.base_point;
	end_point_1_sum += l.end_point_1;
	end_point_2_sum += l.end_point_2;
	line_direction_sum += l.line_direction;
	point_on_line_sum += l.point_on_line;
	length_sum += l.length;
  }

  size_t sz = this->history.size();
  this->smooth.base_point = base_point_sum / sz;
  this->smooth.cloud = linfo.cloud;
  this->smooth.end_point_1 = end_point_1_sum / sz;
  this->smooth.end_point_2 = end_point_2_sum / sz;
  this->smooth.length = length_sum / sz;
  this->smooth.line_direction = line_direction_sum / sz;
  this->smooth.point_on_line = point_on_line_sum / sz;

  Eigen::Vector3f x_axis(1,0,0);

  Eigen::Vector3f ld_unit = this->smooth.line_direction / this->smooth.line_direction.norm();
  Eigen::Vector3f pol_invert = Eigen::Vector3f(0,0,0) - this->smooth.point_on_line;
  Eigen::Vector3f P = this->smooth.point_on_line + pol_invert.dot(ld_unit) * ld_unit;
  this->smooth.bearing = std::acos(x_axis.dot(P) / P.norm());
  // we also want to encode the direction of the angle
  if (P[1] < 0)
    this->smooth.bearing = std::abs(this->smooth.bearing) * -1.;

  Eigen::Vector3f l_diff = raw.end_point_2 - raw.end_point_1;
  Eigen::Vector3f l_ctr = raw.end_point_1 + l_diff / 2.;
  this->bearing_center = std::acos(x_axis.dot(l_ctr) / l_ctr.norm());
  if (l_ctr[1] < 0)
    this->bearing_center = std::abs(this->bearing_center) * -1.;
}



