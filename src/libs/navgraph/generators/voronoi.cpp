
/***************************************************************************
 *  voronoi.cpp - generate navgraph from Voronoi using CGAL
 *
 *  Created: Tue Jan 13 11:53:29 2015
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

#include <navgraph/generators/voronoi.h>
#include <core/exception.h>

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>

// typedefs for defining the adaptor
typedef CGAL::Exact_predicates_inexact_constructions_kernel                  K;
typedef CGAL::Delaunay_triangulation_2<K>                                    DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT,AT,AP>                                    VD;

// typedef for the result type of the point location
typedef AT::Site_2                    Site_2;
typedef AT::Point_2                   Point_2;

typedef VD::Locate_result             Locate_result;
typedef VD::Vertex_handle             Vertex_handle;
typedef VD::Face_handle               Face_handle;
typedef VD::Halfedge_handle           Halfedge_handle;
typedef VD::Ccb_halfedge_circulator   Ccb_halfedge_circulator;

typedef K::Iso_rectangle_2            Iso_rectangle;

typedef std::map<std::string, Point_2> Point_map;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphGeneratorVoronoi <navgraph/generators/voronoi.h>
 * Generate navgraph using a Voronoi diagram.
 * @author Tim Niemueller
 */

/** Default constructor. */
NavGraphGeneratorVoronoi::NavGraphGeneratorVoronoi()
  : bbox_enabled_(false),
    bbox_p1_x_(0.), bbox_p1_y_(0.), bbox_p2_x_(0.), bbox_p2_y_(0.)

{
}

/** Constructor with bounding box.
 * This constructor will cause compute() to ignore any edge with a vertex
 * out of the given bounding box area.
 * @param bbox_p1_x X coordinate of first (lower) bounding box point
 * @param bbox_p1_y y coordinate of first (lower) bounding box point
 * @param bbox_p2_x X coordinate of second (upper) bounding box point
 * @param bbox_p2_y y coordinate of second (upper) bounding box point
 */
NavGraphGeneratorVoronoi::NavGraphGeneratorVoronoi(float bbox_p1_x, float bbox_p1_y,
						   float bbox_p2_x, float bbox_p2_y)
  : bbox_enabled_(true),
    bbox_p1_x_(bbox_p1_x), bbox_p1_y_(bbox_p1_y),
    bbox_p2_x_(bbox_p2_x), bbox_p2_y_(bbox_p2_y)
{
}


/** Destructor. */
NavGraphGeneratorVoronoi::~NavGraphGeneratorVoronoi()
{
}


/** Check if a point is already contained in a map.
 * @param points map of points to check for @p point
 * @param point point to check whether it already exists
 * @param name if the point was found in the map will be assigned
 * the name of the point in the map upon return
 * @return true if the point has been found in the map, false otherwise
 */
static bool
contains(Point_map points, Point_2 point, std::string &name)
{
  for (auto p : points) {
    K::FT dist = sqrt(CGAL::squared_distance(p.second, point));
    if (dist < std::numeric_limits<K::FT>::epsilon()) {
      name = p.first;
      return true;
    }
  }
  return false;
}


/** Generate a new name
 * @param i number parameter for point name, will be incremented by one
 * @return string with a new point name
 */
static std::string
genname(unsigned int &i)
{
  char * name;
  if (asprintf(&name, "V_%02u", ++i) != -1) {
    std::string rv = name;
    free(name);
    return rv;
  } else {
    throw Exception("Failed to create node name");
  }
}


/** Add an obstacle point.
 * An obstacle point will be the representative for a Voronoi
 * face in the newly generated graph.
 * @param x X coordinate of point
 * @param y Y coordinate of point
 */
void
NavGraphGeneratorVoronoi::add_obstacle(float x, float y)
{
  obstacles_.push_back(std::make_pair(x, y));
}


/** Clear all obstacle points. */
void
NavGraphGeneratorVoronoi::clear()
{
  obstacles_.clear();
}


/** Compute graph.
 * @param graph the resulting nodes and edges will be added to this graph.
 * The graph will *not* be cleared automatically. The graph will be locked
 * while adding nodes.
 */
void
NavGraphGeneratorVoronoi::compute(fawkes::LockPtr<fawkes::NavGraph> graph)
{
  VD vd;
  for (auto o : obstacles_) {
    vd.insert(Site_2(o.first, o.second));
  }

  Iso_rectangle rect(Point_2(bbox_p1_x_, bbox_p1_y_), Point_2(bbox_p2_x_, bbox_p2_y_));

  std::map<std::string, Point_2> points;
  std::map<std::string, std::string> props_gen;
  props_gen["generated"] = "true";

  unsigned int num_nodes = 0;
  if (vd.is_valid()) {
    VD::Edge_iterator e;
    graph.lock();
    for (e = vd.edges_begin(); e != vd.edges_end(); ++e) {
      if (e->is_segment()) {
	if (bbox_enabled_) {
	  CGAL::Bounded_side source_side, target_side;
	  source_side = rect.bounded_side(e->source()->point());
	  target_side = rect.bounded_side(e->target()->point());

	  if (source_side == CGAL::ON_UNBOUNDED_SIDE || target_side == CGAL::ON_UNBOUNDED_SIDE)
	    continue;
	}

	// check if we have a point in the vicinity
	std::string source_name, target_name;
	bool have_source = contains(points, e->source()->point(), source_name);
	bool have_target = contains(points, e->target()->point(), target_name);

	if (! have_source) {
	  source_name = genname(num_nodes);
	  //printf("Adding source %s\n", source_name.c_str());
	  graph->add_node(NavGraphNode(source_name,
				       e->source()->point().x(), e->source()->point().y(),
				       props_gen));
	  points[source_name] = e->source()->point();
	}
	if (! have_target) {
	  target_name = genname(num_nodes);
	  //printf("Adding target %s\n", target_name.c_str());
	  graph->add_node(NavGraphNode(target_name,
				       e->target()->point().x(), e->target()->point().y(),
				       props_gen));
	  points[target_name] = e->target()->point();
	}

	graph->add_edge(NavGraphEdge(source_name, target_name, props_gen));
      } else {
	//printf("Unbounded edge\n");
      }
    }

    graph->calc_reachability();
    graph.unlock();
  }
}

} // end of namespace fawkes
