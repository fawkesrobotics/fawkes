
/***************************************************************************
 *  generator.cpp - navgraph generator interface and base class
 *
 *  Created: Thu Mar 16 11:00:22 2017
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

#include <core/exception.h>

#include <navgraph/generators/generator.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphGenerator <navgraph/generators/generator.h>
 * Base class for navgraph generators.
 * This class cannot be instantiated and used directly, rather it defines the
 * general interface of specific generators.
 *
 * To implementers of generators this provides some basic interface to
 * define a bounding box, a near threshold, and to add obstacles. Note that
 * these features may not be used by some generators. See the respective
 * documentation.
 *
 * If a bounding box is provided, it shall be defined befined on the ground
 * plane in a right-handed coordinate system (the same in which all obstacles
 * are defined) by two points P1 and P2. The point P1 shall be the "lower left"
 * corner and P2 the "upper right" corner, i.e., P1 < P2, as in P1.x < P2.x and
 * P1.y < P2.y.
 *
 * @fn void NavGraphGenerator::compute(fawkes::LockPtr<fawkes::NavGraph> graph) = 0
 * Compute graph.
 * @param graph the resulting nodes and edges will be added to this graph.
 * The graph shall *not* be cleared automatically. The generator shall
 * lock the graph as necessary.
 *
 * @author Tim Niemueller
 */

/** Default constructor.
 * Disabled bounding box, set near threshold to 1cm.
 */
NavGraphGenerator::NavGraphGenerator()
	: bbox_enabled_(false),
	  bbox_p1_x_(0.), bbox_p1_y_(0.), bbox_p2_x_(0.), bbox_p2_y_(0.),
	  near_threshold_(0.01)
{
}

/** Parametrized constructor.
 * @param params parameters
 */
NavGraphGenerator::NavGraphGenerator(const std::map<std::string, std::string> params)
	: bbox_enabled_(false),
	  bbox_p1_x_(0.), bbox_p1_y_(0.), bbox_p2_x_(0.), bbox_p2_y_(0.),
	  near_threshold_(0.01), params_(params)
{
}


/** Destructor. */
NavGraphGenerator::~NavGraphGenerator()
{
}



/** Generate a new name
 * @param i number parameter for point name, will be incremented by one
 * @return string with a new point name
 */
std::string
NavGraphGenerator::genname(unsigned int &i)
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


/** Set bounding box.
 * Setting a bounding box will cause compute() to ignore any edge with
 * a vertex out of the given bounding box area.
 * @param bbox_p1_x X coordinate of first (lower) bounding box point
 * @param bbox_p1_y y coordinate of first (lower) bounding box point
 * @param bbox_p2_x X coordinate of second (upper) bounding box point
 * @param bbox_p2_y y coordinate of second (upper) bounding box point
 */
void
NavGraphGenerator::set_bounding_box(float bbox_p1_x, float bbox_p1_y,
                                    float bbox_p2_x, float bbox_p2_y)
{
	bbox_enabled_ = true;
	bbox_p1_x_ = bbox_p1_x;
	bbox_p1_y_ = bbox_p1_y;
	bbox_p2_x_ = bbox_p2_x;
	bbox_p2_y_ = bbox_p2_y;  
}

/** Set distance threshold for considering nodes to be the same.
 * @param near_threshold distance threshold for which to consider
 * nodes to be the same if the distance is smaller than this
 * threshold.
 */
void
NavGraphGenerator::set_near_threshold(float near_threshold)
{
	near_threshold_ = near_threshold;
}

/** Add an obstacle point.
 * An obstacle point will be the representative for a Voronoi
 * face in the newly generated graph.
 * @param x X coordinate of point
 * @param y Y coordinate of point
 */
void
NavGraphGenerator::add_obstacle(float x, float y)
{
	obstacles_.push_back(std::make_pair(x, y));
}

} // end of namespace fawkes
