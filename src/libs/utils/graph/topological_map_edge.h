
/***************************************************************************
 *  topological_map_edge.h - Topological graph edge
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

#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TopologicalMapEdge {
 public:
  TopologicalMapEdge();

  TopologicalMapEdge(std::string from, std::string to,
                     std::map<std::string, std::string> properties,
                     bool directed = false);

  TopologicalMapEdge(std::string from, std::string to,
                     bool directed = false);

  /** Get edge originating node name.
   * @return edge originating node name */
  const std::string &  from() const
  { return from_; }

  /** Get edge target node name.
   * @return edge target node name */
  const std::string &  to() const
  { return to_; }

  void set_from(std::string from);
  void set_to(std::string to);
  void set_directed(bool directed);

  /** Get all properties.
   * @return property map
   */
  std::map<std::string, std::string> &  properties()
  { return properties_; }

  /** Check if node has specified property.
   * @param property property key
   * @return true if node has specified property, false otherwise
   */
  bool has_property(std::string property)
  { return properties_.find(property) != properties_.end(); }

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


  std::string property(std::string prop);

  /** Get property converted to float.
   * @param prop property key
   * @return property value
   */
  float property_as_float(std::string prop)
  { return StringConversions::to_float(property(prop)); }

  /** Get property converted to int.
   * @param prop property key
   * @return property value
   */
  int property_as_int(std::string prop)
  { return StringConversions::to_int(property(prop)); }

  /** Get property converted to bol.
   * @param prop property key
   * @return property value
   */
  bool property_as_bool(std::string prop)
  { return StringConversions::to_bool(property(prop)); }

  /** Check edges for equality.
   * Edges are equal if they have the same origination and destination
   * nodes and the same directed status.
   * @param e edge to compare with
   * @return true if the node is the same as this one, false otherwise
   */
  bool operator==(const TopologicalMapEdge &e) const
  { return from_ == e.from_ && to_ == e.to_ && directed_ == e.directed_; }


  /** Less than operator based on node from and to names.
   * One edge is less than another if this is true for their respective names.
   * @param e edge to compare with
   * @return true if this edge is less than the given one
   */
  bool operator<(const TopologicalMapEdge &e) const
  { return (from_ == e.from_ && to_ < e.to_) || (from_ < e.from_); }

 private:
  std::string from_;
  std::string to_;
  bool directed_;
  std::map<std::string, std::string> properties_;
};

} // end of namespace fawkes

#endif
