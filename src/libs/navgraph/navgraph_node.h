
/***************************************************************************
 *  navgraph_node.h - Topological graph node
 *
 *  Created: Fri Sep 21 16:01:26 2012
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

#ifndef __UTILS_GRAPH_TOPOLOGICAL_MAP_NODE_H_
#define __UTILS_GRAPH_TOPOLOGICAL_MAP_NODE_H_

#include <utils/misc/string_conversions.h>

#include <map>
#include <vector>
#include <string>
#include <cmath>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphNode {
 public:
  NavGraphNode();

  NavGraphNode(const std::string &name, float x, float y,
                     std::map<std::string, std::string> properties);

  NavGraphNode(const std::string &name, float x, float y);

  /** Get name of node.
   * @return name of node */
  const std::string &  name() const
  { return name_; }

  /** Get X coordinate in global frame.
   * @return X coordinate in global frame */
  float x() const
  { return x_; }

  /** Get Y coordinate in global frame.
   * @return Y coordinate in global frame */
  float y() const
  { return y_; }

  /** Check if this node shall be unconnected.
   * @return true if the node is unconnected, false otherwise */
  bool unconnected() const
  { return unconnected_; }

  void set_x(float x);
  void set_y(float y);
  void set_name(const std::string &name);
  void set_unconnected(bool unconnected);

  /** Get euclidean distance from this node to another.
   * @param n node to get distance to
   * @return distance */
  float distance(const NavGraphNode &n)
  { return sqrtf(powf(x_ - n.x_, 2) + powf(y_ - n.y_, 2)); }

  /** Get euclidean distance from this node to a point.
   * @param x point X coordinate
   * @param y point Y coordinate
   * @return distance */
  float distance(float x, float y)
  { return sqrtf(powf(x_ - x, 2) + powf(y_ - y, 2)); }

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

  /** Check if node is valid, i.e. it has a name.
   * @return true if node is valid, false otherwise
   */
  bool is_valid() const
  { return name_ != ""; }

  void set_properties(const std::map<std::string, std::string> &properties);
  void set_property(const std::string &property, const std::string &value);
  void set_property(const std::string &property, float value);
  void set_property(const std::string &property, int value);
  void set_property(const std::string &property, bool value);

  std::string property(const std::string &prop) const;

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

  /** Check nodes for equality.
   * Nodes are equal if they have the same name.
   * @param n node to compare with
   * @return true if the node is the same as this one, false otherwise
   */
  bool operator==(const NavGraphNode &n) const
  { return name_ == n.name_; }

  /** Check nodes for inequality.
   * Nodes are inequal if they have different names.
   * @param n node to compare with
   * @return true if the node is different from this one, false otherwise
   */
  bool operator!=(const NavGraphNode &n) const
  { return name_ != n.name_; }

  /** Check if node is valid.
   * A node is valid if it has a name set.
   * @return true if the node is valid, false otherwise
   */
  operator bool() const
  { return name_ != ""; }

  void set_reachable_nodes(std::vector<std::string> reachable_nodes);

  /** Get reachable nodes.
   * @return vector of directly reachable nodes.
   */
  const std::vector<std::string> &  reachable_nodes() const
  { return reachable_nodes_; }
    
 private:
  std::string name_;
  float       x_;
  float       y_;
  bool        unconnected_;
  std::map<std::string, std::string> properties_;
  std::vector<std::string> reachable_nodes_;
};


} // end of namespace fawkes

#endif
