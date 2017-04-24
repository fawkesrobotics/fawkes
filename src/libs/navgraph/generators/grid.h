
/***************************************************************************
 *  grid.h - generate navgraph based on a grid
 *
 *  Created: Sun Apr 23 18:22:09 2017
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

#ifndef __LIBS_NAVGRAPH_GENERATOR_GRID_H_
#define __LIBS_NAVGRAPH_GENERATOR_GRID_H_

#include <navgraph/navgraph.h>
#include <navgraph/generators/generator.h>

#include <utils/math/polygon.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphGeneratorGrid : public NavGraphGenerator
{
 public:
	NavGraphGeneratorGrid(const std::map<std::string, std::string> &params);
	virtual ~NavGraphGeneratorGrid();

	virtual void compute(fawkes::LockPtr<fawkes::NavGraph> graph);

 private:
	float spacing_;
	float margin_;
	bool  add_diagonals_;
};

} // end of namespace fawkes

#endif
