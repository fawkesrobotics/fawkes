
/***************************************************************************
 *  shapemodel.cpp - Abstract class defining a shape model
 *
 *  Created: Wed Mar 21 17:53:39 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2005       Martin Heracles <Martin.Heracles@rwth-aachen.de>
 *             2005       Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#include <fvmodels/shape/shapemodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Shape <fvmodels/shape/shapemodel.h>
 * Shape interface.
 * Generic API for accessing using shapes.
 *
 * @fn void Shape::setMargin(unsigned int margin)
 * Set margin around shape
 * @param margin margin
 *
 * @fn bool Shape::isClose(unsigned int in_roi_x, unsigned int in_roi_y)
 * Check if the given point is close to the shape.
 * @param in_roi_x x coordinate of point in the same ROI as the shape
 * @param in_roi_y y coordinate of point in the same ROI as the shape
 * @return true if point is close to shape, false otherwise
 */

/** Virtual empty destructor. */
Shape::~Shape()
{
}


/** @class ShapeModel <fvmodels/shape/shapemodel.h>
 * Shape model interface.
 *
 *
 * @fn std::string ShapeModel::getName(void) const
 * Get name of shape model.
 * @return name of shape model.
 *
 * @fn int ShapeModel::parseImage( unsigned char* buffer, ROI *roi)
 * Parse image for given ROI.
 * @param buffer image buffer
 * @param roi ROI
 * @return number of shapes found
 *
 * @fn int ShapeModel::getShapeCount(void) const
 * Get number of shapes.
 * @return number of shapes.
 *
 * @fn Shape* ShapeModel::getShape(int id) const
 * Get specific shape.
 * @param id shape ID
 * @return shape, do NOT free!
 *
 * @fn Shape* ShapeModel::getMostLikelyShape(void) const
 * Get best candidate.
 * @return best candidate shape, do not free.
 */

/** Virtual empty destructor. */
ShapeModel::~ShapeModel()
{
}

} // end namespace firevision
