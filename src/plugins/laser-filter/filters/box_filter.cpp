
/***************************************************************************
 *  map_filter.cpp - Laser map data filter
 *
 *  Created: Fri Jul 17 20:38:14 2015
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
 *                  2015  Tobias Neumann
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

#include "box_filter.h"

#include <core/exception.h>
#include <utils/time/time.h>
#include <utils/math/coord.h>
#include <cmath>
#include <string>
#include <limits>

using namespace fawkes;

/** @class LaserBoxFilterDataFilter "box_filter.h
 * Removes laser data which is represented by a set of boxes.
 * These boxes can be defined by an interface.
 * @author Nicolas Limpert
 */

/** Constructor.
 * @param filter_name name of this filter
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 * @param tf_listener to access the tf::Transformer aspect
 * @param config to access the Configuration aspect
 * @param logger to access the Logger aspect
 * @param blackboard to open the LaserBoxFilterInterface for writing
 */
LaserBoxFilterDataFilter::LaserBoxFilterDataFilter(const std::string filter_name,
                                                   unsigned int in_data_size,
                                                   std::vector<LaserDataFilter::Buffer *> &in,
                                                   fawkes::tf::Transformer *tf_listener,
                                                   fawkes::Configuration *config,
                                                   fawkes::Logger *logger, BlackBoard *blackboard)
  : LaserDataFilter(filter_name, in_data_size, in, 1)
{
  tf_listener_ = tf_listener;
  config_ = config;
  logger_ = logger;
  frame_map_ = config_->get_string("/frames/fixed");
  cfg_occupied_thresh_ = std::numeric_limits<float>::max();
  box_filter_if_ = blackboard->open_for_writing<fawkes::LaserBoxFilterInterface>("Laser Box Filter");
}

/** Returns whether the given coordinates are within one of the boxes or not.
 *  @param x the x position
 *  @param y the y position
 *  @return true if the point is within a box
 *          false otherwise
 */
bool
LaserBoxFilterDataFilter::point_in_rectangle(float x, float y)
{
  // Test if given point x, y is inside rectangle is given by:
  //
  // 0 <= dot(AB,AM) <= dot(AB,AB) && 0 <= dot(BC,BM) <= dot(BC,BC)
  //
  // This approach is based on https://codepen.io/mattburns/pen/jrrprN

  Vector point;
  point.x = x;
  point.y = y;

  bool is_in_rect = false;

  for (std::vector<Box>::iterator it = boxes_.begin(); is_in_rect == false && it != boxes_.end(); it++) {
    Vector AB = d_vec(it->a, it->b);
    Vector AM = d_vec(it->a, point);
    Vector BC = d_vec(it->b, it->c);
    Vector BM = d_vec(it->b, point);
    double dotABAM = dot(AB, AM);
    double dotABAB = dot(AB, AB);
    double dotBCBM = dot(BC, BM);
    double dotBCBC = dot(BC, BC);
    is_in_rect = 0 <= dotABAM && dotABAM <= dotABAB && 0 <= dotBCBM && dotBCBM <= dotBCBC;
  }
  return is_in_rect;
}


LaserBoxFilterDataFilter::Vector
LaserBoxFilterDataFilter::d_vec(LaserBoxFilterDataFilter::Vector p1,
                                LaserBoxFilterDataFilter::Vector p2)
{
  LaserBoxFilterDataFilter::Vector ret_val;
  ret_val.x = (p2.x - p1.x);
  ret_val.y = (p2.y - p1.y);
  return ret_val;
}

inline double
LaserBoxFilterDataFilter::dot(LaserBoxFilterDataFilter::Vector u,
                              LaserBoxFilterDataFilter::Vector v)
{
  return u.x * v.x + u.y * v.y;
}


void
LaserBoxFilterDataFilter::filter()
{
  while (! box_filter_if_->msgq_empty() ) {
    if (box_filter_if_->msgq_first_is<LaserBoxFilterInterface::CreateNewBoxFilterMessage>()) {
      LaserBoxFilterInterface::CreateNewBoxFilterMessage *msg = box_filter_if_->msgq_first(msg);

      Box new_box;
      new_box.a.x = msg->p1(0);
      new_box.a.y = msg->p1(1);
      new_box.b.x = msg->p2(0);
      new_box.b.y = msg->p2(1);
      new_box.c.x = msg->p3(0);
      new_box.c.y = msg->p3(1);
      new_box.d.x = msg->p4(0);
      new_box.d.y = msg->p4(1);

      boxes_.push_back(new_box);
      box_filter_if_->set_num_boxes(box_filter_if_->num_boxes() + 1);
      box_filter_if_->write();
    }

    box_filter_if_->msgq_pop();
  }

  const unsigned int vecsize = in.size();
  if (vecsize == 0)  return;

  for (unsigned int a = 0; a < vecsize; ++a) {
    // get tf to map of laser input
    fawkes::tf::StampedTransform transform;
    try{
      tf_listener_->lookup_transform(frame_map_.c_str(), in[a]->frame, *(in[a]->timestamp), transform);
    } catch(fawkes::tf::TransformException &e) {
      try{
        tf_listener_->lookup_transform(frame_map_.c_str(), in[a]->frame, fawkes::Time(0, 0), transform);
      } catch(fawkes::tf::TransformException &e) {
        logger_->log_warn("box_filter", "Can't transform laser-data (%s -> %s)",
                          frame_map_.c_str(), in[a]->frame.c_str());
        return;
      }
    }
    // set out meta info
    out[a]->frame = in[a]->frame;
    out[a]->timestamp = in[a]->timestamp;
    // for each point
    for (unsigned int i = 0; i < out_data_size; ++i) {
      bool add = true;
      // check nan
      if ( std::isfinite(in[a]->values[i]) ) {
        // transform to cartesian
        double angle = M_PI * (360.f / out_data_size * i ) / 180;

        float x, y;
        fawkes::polar2cart2d(angle, in[a]->values[i], &x, &y);

        // transform into map
        fawkes::tf::Point p;
        p.setValue(x, y, 0.);
        p = transform * p;

        add = !point_in_rectangle(p.getX(), p.getY());

      }
      if (add) {
        out[a]->values[i] = in[a]->values[i];
      } else {
        out[a]->values[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
}
