
/***************************************************************************
 *  voronoi.h - generate navgraph from Voronoi using CGAL
 *
 *  Created: Mon Jan 12 21:31:38 2015
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

#ifndef __LIBS_NAVGRAPH_GENERATOR_VORONOI_H_
#define __LIBS_NAVGRAPH_GENERATOR_VORONOI_H_

#include <navgraph/navgraph.h>
#include <navgraph/generators/generator.h>

#include <utils/math/polygon.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphGeneratorVoronoi : public NavGraphGenerator
{
 public:
	NavGraphGeneratorVoronoi();
	virtual ~NavGraphGeneratorVoronoi();

	virtual void compute(fawkes::LockPtr<fawkes::NavGraph> graph);

	/** Get list of polygons.
	 * @return list of polygons, each polygon contains the vertices of a
	 * bounded face of the Voronoi diagram.
	 */
	const std::list<Polygon2D> &  face_polygons() const
	{ return polygons_; }

 private:
	std::list<Polygon2D> polygons_;
};

} // end of namespace fawkes

#endif
