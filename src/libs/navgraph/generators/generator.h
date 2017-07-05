
/***************************************************************************
 *  generator.h - navgraph generator interface and base class
 *
 *  Created: Thu Mar 16 10:56:32 2017
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

#ifndef __LIBS_NAVGRAPH_GENERATOR_GENERATOR_H_
#define __LIBS_NAVGRAPH_GENERATOR_GENERATOR_H_

#include <navgraph/navgraph.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NavGraphGenerator
{
 public:
	NavGraphGenerator();
	NavGraphGenerator(std::map<std::string, std::string> params);
	virtual ~NavGraphGenerator();

	virtual void compute(fawkes::LockPtr<fawkes::NavGraph> graph) = 0;

	virtual void set_bounding_box(float bbox_p1_x, float bbox_p1_y,
	                              float bbox_p2_x, float bbox_p2_y);
	virtual void set_near_threshold(float near_threshold);
	virtual void add_obstacle(float x, float y);

	static std::string genname(unsigned int &i);
	
 protected:
	bool  bbox_enabled_;   ///< True if bounding box requested, false otherwise
	float bbox_p1_x_;      ///< X part of P1 for bounding box
	float bbox_p1_y_;      ///< Y part of P1 for bounding box
	float bbox_p2_x_;      ///< X part of P2 for bounding box
	float bbox_p2_y_;      ///< Y part of P2 for bounding box
	float near_threshold_; ///< distance threshold when to consider two nodes to be the same

	/// Obstacles to consider during navgraph generation.
	std::list<std::pair<float, float>> obstacles_;
  /// Parameters specific to the actual generator in a generic format.
	std::map<std::string, std::string> params_;
};

} // end of namespace fawkes

#endif
