
/***************************************************************************
 *  grid.cpp - generate navgraph based on grid
 *
 *  Created: Sun Apr 23 18:46:12 2017
 *  Copyright  2015-2017  Tim Niemueller [www.niemueller.de]
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

#include <navgraph/generators/grid.h>
#include <core/exception.h>
#include <core/threading/mutex_locker.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Sweep_line_2_algorithms.h>
#include <CGAL/Cartesian.h>

typedef CGAL::Quotient<CGAL::MP_Float>                  NT;
typedef CGAL::Cartesian<NT>                             Kernel;

typedef Kernel::Point_2                                 Point;
typedef Kernel::Segment_2                               Segment;
typedef Kernel::Vector_2                                Vector_2;
typedef CGAL::Points_on_segment_2<Point>                PG;
typedef CGAL::Creator_uniform_2< Point, Segment>        Creator;
typedef CGAL::Join_input_iterator_2< PG, PG, Creator>   Segm_iterator;
typedef CGAL::Counting_iterator<Segm_iterator,Segment>  Count_iterator;
typedef std::vector<Segment>                            Vector;


typedef CGAL::Arr_segment_traits_2<Kernel>              Traits_2;
typedef Traits_2::Curve_2                               Curve_2;


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphGeneratorGrid <navgraph/generators/grid.h>
 * Generate navgraph using a Grid diagram.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param params algorithm parameters. Valid parameters are:
 * - spacing (float): inter-cell midpoint spacing (required).
 */
NavGraphGeneratorGrid::NavGraphGeneratorGrid(const std::map<std::string, std::string> &params)
{
	if (params.find("spacing") == params.end()) {
		throw Exception("Grid algorithm requires 'spacing' parameter");
	}
	if (params.find("margin") == params.end()) {
		throw Exception("Grid algorithm requires 'margin' parameter");
	}
	add_diagonals_ = false;
	if (params.find("add-diagonals") != params.end()) {
		add_diagonals_ = (params.at("add-diagonals") == "true");
	}
	try {
		std::size_t pos;
		spacing_ = std::stof(params.at("spacing"), &pos);
		if (pos < params.at("spacing").size()) {
			throw Exception("Cannot convert spacing '%s' to float", params.at("spacing").c_str());
		}
	} catch (std::invalid_argument &e) {
		throw Exception("%s", e.what());
	} catch (std::out_of_range &e) {
		throw Exception("%s", e.what());
	}

	try {
		std::size_t pos;
		margin_ = std::stof(params.at("margin"), &pos);
		if (pos < params.at("margin").size()) {
			throw Exception("Cannot convert margin '%s' to float", params.at("margin").c_str());
		}
	} catch (std::invalid_argument &e) {
		throw Exception("%s", e.what());
	} catch (std::out_of_range &e) {
		throw Exception("%s", e.what());
	}
}

/** Destructor. */
NavGraphGeneratorGrid::~NavGraphGeneratorGrid()
{
}


void
NavGraphGeneratorGrid::compute(fawkes::LockPtr<fawkes::NavGraph> graph)
{
	if (! bbox_enabled_) {
		throw Exception("Grid algorithm requires bounding box");
	}

	Point bb_p1(bbox_p1_x_, bbox_p1_y_);
	Point bb_p2(bbox_p2_x_, bbox_p2_y_);

	if (CGAL::compare_xy<Kernel>(bb_p1, bb_p2) != CGAL::SMALLER) {
		throw Exception("Bounding box P1 must be smaller than P2 (x1 < x2 or x1 == x2 and y1 < y2)");
	}

	MutexLocker lock(graph.objmutex_ptr());

	// Calculate number of points in X and Y direction
	// based on bounding box and grid spacing
	const Vector_2 diff(bb_p2 - bb_p1);
	if (diff.x() < spacing_ || diff.y() < spacing_) {
		throw Exception("Bounding box too small for given spacing");
	}
	size_t nx = CGAL::to_double(diff.x()) / spacing_;
	size_t ny = CGAL::to_double(diff.y()) / spacing_;


	// Spacing half, will be used to make sure grid intersection points
	// have at least half spacing margin from bounding box.
	const float spacing_2 = spacing_ / 2;

	
	// Generate HORIZONTAL lines of grid (along x-axis)
	PG p1( Point(bb_p1.x(), bb_p1.y() + spacing_2),
	       Point(bb_p1.x(), bb_p2.y() - spacing_2), ny);
	PG p2( Point(bb_p2.x(), bb_p1.y() + spacing_2),
	       Point(bb_p2.x(), bb_p2.y() - spacing_2), ny);
	Segm_iterator  t1(p1, p2);                     // Segment generator.
	Count_iterator t1_begin(t1);                   // Finite range.
	Count_iterator t1_end(t1, ny);

	// std::for_each(t1_begin, t1_end,
	//               [](const Segment &s) {
	// 	              printf("Segment H (%f,%f) -> (%f,%f)\n",
	// 	                     CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
	// 	                     CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()));
	//               });

	
	// Generate VERTICAL lines of grid (along y-axis)
	PG p3( Point(bb_p1.x() + spacing_2, bb_p1.y()),
	       Point(bb_p2.x() - spacing_2, bb_p1.y()), nx);
	PG p4( Point(bb_p1.x() + spacing_2, bb_p2.y()),
	       Point(bb_p2.x() - spacing_2, bb_p2.y()), nx);
	Segm_iterator  t2(p3, p4);                     // Segment generator.
	Count_iterator t2_begin(t2);                   // Finite range.
	Count_iterator t2_end(t2, nx);

	// std::for_each(t2_begin, t2_end,
	//               [](const Segment &s) {
	// 	              printf("Segment V (%f,%f) -> (%f,%f)\n",
	// 	                     CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
	// 	                     CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()));
	//               });

	// Transform segments to curves for intersection calculation
	std::vector<Curve_2> segs;
	segs.reserve(nx + ny);
	std::transform(t1_begin, t1_end, std::back_inserter(segs),
	               [](const Segment &s) -> Curve_2 {
		               return Curve_2(s.source(), s.target());
	               });
	std::transform(t2_begin, t2_end, std::back_inserter(segs),
	               [](const Segment &s) -> Curve_2 {
		               return Curve_2(s.source(), s.target());
	               });

  // Compute intersection points on grid, these will be the navgraph nodes
  std::vector<Point> ipts;
  ipts.reserve(nx * ny);
  CGAL::compute_intersection_points(segs.begin(), segs.end(), std::back_inserter(ipts));

  // Sort by Y, then by X (relevant for edge generation later to use
  // simple index operations to find predecessor row to connect to)
	std::sort(ipts.begin(), ipts.end(),
	          [](const Point &p1, const Point &p2) {
		          return CGAL::compare_yx<Kernel>(p1, p2) == CGAL::SMALLER;
	          });

	// printf("Num intersections: %zu\n", ipts.size());
	// std::for_each(ipts.begin(), ipts.end(),
	//               [](const Point &p) {
	// 	              printf("Intersection (%f,%f)\n",
	// 	                     CGAL::to_double(p.x()), CGAL::to_double(p.y()));
	//               });

	std::map<std::string, std::string> props_gen = { {"generated", "true" } };

	// Add nodes to graph
  std::vector<std::pair<std::string, Point>> points;
  points.reserve(ipts.size());
  for (size_t i = 0; i < ipts.size(); ++i) {
	  std::string name = "G-" + std::to_string((i % nx) + 1) + "-" + std::to_string((i / nx) + 1);
	  points.push_back(std::make_pair(name, ipts[i]));
	  graph->add_node(NavGraphNode(name, CGAL::to_double(ipts[i].x()), CGAL::to_double(ipts[i].y()), props_gen));
  }

  // std::for_each(points.begin(), points.end(),
  //               [](const auto &p) {
	//                 printf("Point '%s' (%f,%f)\n", p.first.c_str(),
	//                        CGAL::to_double(p.second.x()), CGAL::to_double(p.second.y()));
  //               });

  // Add edges to graph
  for (size_t i = 0; i < points.size(); ++i) {
	  if (i % nx > 0) {
		  // connect to previous node unless it's the first node in the row
		  graph->add_edge(NavGraphEdge(points[i-1].first, points[i].first, props_gen));
	  }
		  
	  if (i >= nx) {
		  // connect to other row, unless it's the first row
		  const size_t prev_i = (((i / nx) - 1) * nx) + (i % nx);
		  graph->add_edge(NavGraphEdge(points[prev_i].first, points[i].first));

		  if (add_diagonals_) {
			  if (prev_i % nx > 0) {
				  graph->add_edge(NavGraphEdge(points[prev_i - 1].first, points[i].first),
				                  NavGraph::EDGE_SPLIT_INTERSECTION);
			  }
			  if (prev_i % nx < (nx - 1)) {
				  graph->add_edge(NavGraphEdge(points[prev_i + 1].first, points[i].first),
				                  NavGraph::EDGE_SPLIT_INTERSECTION);
			  }
		  }
	  }
  }


  // Eliminate edges near obstacles
  const std::vector<NavGraphEdge> &edges = graph->edges();

  for (const auto &o : obstacles_) {
	  std::list<NavGraphEdge> to_remove;
	  for (const NavGraphEdge &e : edges) {
		  try {
			  if (e.distance(o.first, o.second) <= margin_) {
				  to_remove.push_back(e);
			  }
		  } catch (Exception &e) {} // ignore, point is not on edge
	  }
	  std::for_each(to_remove.begin(), to_remove.end(),
	                [&graph](const NavGraphEdge &e) { graph->remove_edge(e); });
  }

  // Orphan nodes are removed by the navgraph-generator thread
}


} // end of namespace fawkes
