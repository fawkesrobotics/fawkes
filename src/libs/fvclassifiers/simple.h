
/***************************************************************************
 *  simple.h - Header for ReallySimpleClassifier
 *
 *  Created: Wed May 18 11:39:10 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef _FIREVISION_CLASSIFIERS_SIMPLE_H_
#define _FIREVISION_CLASSIFIERS_SIMPLE_H_

#include <fvclassifiers/classifier.h>
#include <fvutils/base/types.h>

namespace firevision {

class ScanlineModel;
class ColorModel;

class SimpleColorClassifier : public Classifier
{
public:
	SimpleColorClassifier(ScanlineModel *scanline_model,
	                      ColorModel    *color_model,
	                      unsigned int   min_num_points          = 6,
	                      unsigned int   box_extent              = 50,
	                      bool           upward                  = false,
	                      unsigned int   neighbourhood_min_match = 8,
	                      unsigned int   grow_by                 = 10,
	                      color_t        color                   = C_ORANGE);

	virtual std::list<ROI> *classify();

	virtual void get_mass_point_of_color(ROI *roi, fawkes::upoint_t *massPoint);

private:
	unsigned int consider_neighbourhood(unsigned int x, unsigned int y, color_t what);

	unsigned int neighbourhood_min_match;
	unsigned int grow_by;

	bool         modified;
	unsigned int min_num_points;
	unsigned int box_extent;

	bool upward;

	ScanlineModel *scanline_model;
	ColorModel    *color_model;

	const color_t color;
};

} // end namespace firevision

#endif
