
/***************************************************************************
 *  navgraph_edge.cpp - Topological graph
 *
 *  Created: Fri Sep 21 16:11:50 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include <navgraph/navgraph_edge.h>

#include <core/exception.h>
#include <utils/math/lines.h>

#include <Eigen/Geometry>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#if ! EIGEN_VERSION_AT_LEAST(3,2,0)
namespace workaround {
  template<typename Derived>
  static inline bool allFinite(const Eigen::DenseBase<Derived> &d)
  {
    const Derived t = (d.derived()-d.derived());
    return ((t.derived().array()==t.derived().array()).all());
  }
}
#endif

/** @class NavGraphEdge <navgraph/navgraph_edge.h>
 * Topological graph edge.
 * @author Tim Niemueller
 */

/** Constructor for an invalid edge. */
NavGraphEdge::NavGraphEdge()
{
  directed_ = false;
}


/** Constructor.
 * @param from originating node name
 * @param to target node name
 * @param properties properties of the new node
 * @param directed true if the edge is directed, false for bidirectional edges
 */
NavGraphEdge::NavGraphEdge(const std::string &from, const std::string &to,
			   std::map<std::string, std::string> properties,
			   bool directed)
{
  from_ = from;
  to_ = to;
  properties_ = properties;
  directed_ = directed;
}


/** Constructor.
 * @param from originating node name
 * @param to target node name
 * @param directed true if the edge is directed, false for bidirectional edges
 */
NavGraphEdge::NavGraphEdge(const std::string &from, const std::string &to, bool directed)
{
  from_ = from;
  to_ = to;
  directed_ = directed;
}

/** Set originating node name.
 * @param from originating node name
 */
void
NavGraphEdge::set_from(const std::string &from)
{
  from_ = from;
}


/** Set target node name.
 * @param to target node name
 */
void
NavGraphEdge::set_to(const std::string &to)
{
  to_ = to;
}


/** Set nodes.
 * @param from_node originating node
 * @param to_node target node
 */
void
NavGraphEdge::set_nodes(const NavGraphNode &from_node,
			      const NavGraphNode &to_node)
{
  if (from_node.name() != from_) {
    throw Exception("Conflicting originating node names: %s vs. %s",
		    from_node.name().c_str(), from_.c_str());
  }
  if (to_node.name() != to_) {
    throw Exception("Conflicting target node names: %s vs. %s",
		    to_node.name().c_str(), to_.c_str());
  }

  from_node_ = from_node;
  to_node_   = to_node;
}

/** Set directed state.
 * @param directed true if the edge is directed, false for bidirectional edges
 */
void
NavGraphEdge::set_directed(bool directed)
{
  directed_ = directed;
}

/** Get specified property as string.
 * @param prop property key
 * @return property value as string
 */
std::string
NavGraphEdge::property(const std::string &prop) const
{
  std::map<std::string, std::string>::const_iterator p;
  if ((p = properties_.find(prop)) != properties_.end()) {
    return p->second;
  } else {
    return "";
  }
}

/** Overwrite properties with given ones.
 * @param properties map of properties to set
 */
void
NavGraphEdge::set_properties(const std::map<std::string, std::string> &properties)
{
  properties_ = properties;
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphEdge::set_property(const std::string &property, const std::string &value)
{
  properties_[property] = value;
}


/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphEdge::set_property(const std::string &property, const char *value)
{
  properties_[property] = value;
}


/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphEdge::set_property(const std::string &property, float value)
{
  properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphEdge::set_property(const std::string &property, int value)
{
  properties_[property] = StringConversions::to_string(value);
}

/** Set property.
 * @param property property key
 * @param value property value
 */
void
NavGraphEdge::set_property(const std::string &property, bool value)
{
  properties_[property] = value ? "true" : "false";
}

/** Get the point on edge closest to a given point.
 * The method determines a line perpendicular to the edge which goes through
 * the given point, i.e. the point must be within the imaginary line segment.
 * Then the point on the edge which crosses with that perpendicular line
 * is returned.
 * @param x X coordinate of point to get point on edge for
 * @param y Y coordinate of point to get point on edge for
 * @return coordinate of point on edge closest to given point
 * @throw Exception thrown if the point is out of the line segment and
 * no line perpendicular to the edge going through the given point can
 * be found.
 */
cart_coord_2d_t
NavGraphEdge::closest_point_on_edge(float x, float y) const
{
  const Eigen::Vector2f point(x, y);
  const Eigen::Vector2f origin(from_node_.x(), from_node_.y());
  const Eigen::Vector2f target(to_node_.x(), to_node_.y());
  const Eigen::Vector2f direction(target - origin);
  const Eigen::Vector2f direction_norm = direction.normalized();
  const Eigen::Vector2f diff = point - origin;
  const float t = direction.dot(diff) / direction.squaredNorm();

  if (t >= 0.0 && t <= 1.0) {
    // projection of the point onto the edge is within the line segment
    Eigen::Vector2f point_on_line = origin + direction_norm.dot(diff) * direction_norm;
    return cart_coord_2d_t(point_on_line[0], point_on_line[1]);
  }

  throw Exception("Point (%f,%f) is not on edge %s--%s", x, y,
		  from_.c_str(), to_.c_str());
}


/** Get distance of point to closest point on edge.
 * @param x X coordinate of point to get distance to closest point on edge for
 * @param y Y coordinate of point to get distance to closest point on edge for
 * @return distance to the closest point on the edge
 */
float
NavGraphEdge::distance(float x, float y) const
{
	cart_coord_2d_t poe = closest_point_on_edge(x, y);
	Eigen::Vector2f p;
	p[0] = x; p[1] = y;
	Eigen::Vector2f cp;
	cp[0] = poe.x; cp[1] = poe.y;
	return (p - cp).norm();
}


/** Check if the edge intersects with another line segment.
 * @param x1 X coordinate of first point of line segment to test
 * @param y1 Y coordinate of first point of line segment to test
 * @param x2 X coordinate of first point of line segment to test
 * @param y2 Y coordinate of first point of line segment to test
 * @param ip upon returning true contains intersection point,
 * not modified is return value is false
 * @return true if the edge intersects with the line segment, false otherwise
 */
bool
NavGraphEdge::intersection(float x1, float y1, float x2, float y2,
			   fawkes::cart_coord_2d_t &ip) const
{
  const Eigen::Vector2f e_from(from_node_.x(), from_node_.y());
  const Eigen::Vector2f e_to(to_node_.x(), to_node_.y());
  const Eigen::Vector2f p1(x1, y1);
  const Eigen::Vector2f p2(x2, y2);
  const Eigen::Vector2f lip = line_segm_intersection(e_from, e_to, p1, p2);
#if EIGEN_VERSION_AT_LEAST(3,2,0)
  if (lip.allFinite()) {
#else
  if (workaround::allFinite(lip)) {
#endif
    ip.x = lip[0];
    ip.y = lip[1];
    return true;
  } else {
    return false;
  }
}

/** Check if the edge intersects with another line segment.
 * @param x1 X coordinate of first point of line segment to test
 * @param y1 Y coordinate of first point of line segment to test
 * @param x2 X coordinate of first point of line segment to test
 * @param y2 Y coordinate of first point of line segment to test
 * @return true if the edge intersects with the line segment, false otherwise
 */
bool
NavGraphEdge::intersects(float x1, float y1, float x2, float y2) const
{
  const Eigen::Vector2f e_from(from_node_.x(), from_node_.y());
  const Eigen::Vector2f e_to(to_node_.x(), to_node_.y());
  const Eigen::Vector2f p1(x1, y1);
  const Eigen::Vector2f p2(x2, y2);
  return line_segm_intersect(e_from, e_to, p1, p2);
}

} // end of namespace fawkes
