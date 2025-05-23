
/***************************************************************************
 *  shape_remover.h - Header of shape remover
 *
 *  Created: Wed Sep 28 11:25:04 2005
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

#ifndef _FIREVISION_FILTER_SHAPEREMOVER_H_
#define _FIREVISION_FILTER_SHAPEREMOVER_H_

#include <fvfilters/filter.h>

namespace firevision {

class ShapeModel;
class Shape;

class FilterShapeRemover : public Filter
{
public:
	FilterShapeRemover();

	virtual void set_shape(Shape *shape);
	virtual void set_margin(unsigned int margin);
	virtual void apply();

private:
	Shape       *shape;
	unsigned int margin;
};

} // end namespace firevision

#endif
