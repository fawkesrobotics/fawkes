
/***************************************************************************
 *  navgraph_edge.h - Topological graph edge
 *
 *  Created: Fri Sep 21 16:08:27 2012
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

#ifndef __UTILS_GRAPH_TOPOLOGICAL_MAP_EDGE_H_
#define __UTILS_GRAPH_TOPOLOGICAL_MAP_EDGE_H_

#include <utils/misc/string_conversions.h>

#include <navgraph/navgraph_node.h>
#include <utils/math/types.h>

#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraph;

class NavGraphEdge {
  friend NavGraph;

 public:
  NavGraphEdge();

  NavGraphEdge(const std::string &from, const std::string &to,
                     std::map<std::string, std::string> properties,
                     bool directed = false);

  NavGraphEdge(const std::string &from, const std::string &to,
                     bool directed = false);

  /** Get edge originating node name.
   * @return edge originating node name */
  const std::string &  from() const
  { return from_; }

  /** Get edge target node name.
   * @return edge target node name */
  const std::string &  to() const
  { return to_; }


  /** Get edge originating node.
   * @return edge originating node */
  const NavGraphNode &  from_node() const
  { return from_node_; }

  /** Get edge target node.
   * @return edge target node */
  const NavGraphNode &  to_node() const
  { return to_node_; }

  fawkes::cart_coord_2d_t closest_point_on_edge(float x, float y) const;
  float distance(float x, float y) const;
  bool intersects(float x1, float y1, float x2, float y2) const;
  bool intersection(float x1, float y1, float x2, float y2,
		    fawkes::cart_coord_2d_t &ip) const;

  void set_from(const std::string &from);
  void set_to(const std::string &to);
  void set_directed(bool directed);

  /** Get all properties.
   * @return property map
   */
  const std::map<std::string, std::string> &  properties() const
  { return properties_; }

  /** Check if node has specified property.
   * @param property property key
   * @return true if node has specified property, false otherwise
   */
  bool has_property(const std::string &property) const
  { return properties_.find(property) != properties_.end(); }

  void set_properties(const std::map<std::string, std::string> &properties);
  void set_property(const std::string &property, const std::string &value);
  void set_property(const std::string &property, const char *value);
  void set_property(const std::string &property, float value);
  void set_property(const std::string &property, int value);
  void set_property(const std::string &property, bool value);

  /** Get property converted to float.
   * @param prop property key
   * @return property value
   */
  float property_as_float(const std::string &prop) const
  { return StringConversions::to_float(property(prop)); }

  /** Get property converted to int.
   * @param prop property key
   * @return property value
   */
  int property_as_int(const std::string &prop) const
  { return StringConversions::to_int(property(prop)); }

  /** Get property converted to bol.
   * @param prop property key
   * @return property value
   */
  bool property_as_bool(const std::string &prop) const
  { return StringConversions::to_bool(property(prop)); }

  /** Check if edge is valid.
   * An edge is valid iff it has originating and target node name values.
   * @return true if edge is valid, false otherwise
   */
  bool is_valid() const
  { return from_ != "" && to_ != ""; }

  /** Check if edge is directed.
   * @return true if edge is directed, false otherwise.
   */
  bool is_directed() const
  { return directed_; }


  std::string property(const std::string &prop) const;

  /** Get property converted to float.
   * @param prop property key
   * @return property value
   */
  float property_as_float(const std::string &prop)
  { return StringConversions::to_float(property(prop)); }

  /** Get property converted to int.
   * @param prop property key
   * @return property value
   */
  int property_as_int(const std::string &prop)
  { return StringConversions::to_int(property(prop)); }

  /** Get property converted to bol.
   * @param prop property key
   * @return property value
   */
  bool property_as_bool(const std::string &prop)
  { return StringConversions::to_bool(property(prop)); }

  /** Check edges for equality.
   * Edges are equal if they have the same origination and destination
   * nodes and the same directed status.
   * @param e edge to compare with
   * @return true if the node is the same as this one, false otherwise
   */
  bool operator==(const NavGraphEdge &e) const
  { return from_ == e.from_ && to_ == e.to_ && directed_ == e.directed_; }


  /** Less than operator based on node from and to names.
   * One edge is less than another if this is true for their respective names.
   * @param e edge to compare with
   * @return true if this edge is less than the given one
   */
  bool operator<(const NavGraphEdge &e) const
  { return (from_ == e.from_ && to_ < e.to_) || (from_ < e.from_); }


  /** Check of edge is valid.
   * An edge is valid if both the originating and the target node
   * name is set to a non-empty string.
   * @return true if the node is valid, false otherwise
   */
  operator bool() const
  { return from_ != "" && to_ != ""; }

  void set_nodes(const NavGraphNode &from_node, const NavGraphNode &to_node);

 private:
  std::string from_;
  std::string to_;
  bool directed_;
  std::map<std::string, std::string> properties_;

  NavGraphNode from_node_;
  NavGraphNode to_node_;
};

} // end of namespace fawkes

#endif
