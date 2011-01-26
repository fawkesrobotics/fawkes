
/***************************************************************************
 *  shapemodel.h - Abstract class defining a shape model
 *
 *  Created: Tue May 03 19:50:02 2005
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

#ifndef __FIREVISION_MODELS_SHAPE_SHAPEMODEL_H_
#define __FIREVISION_MODELS_SHAPE_SHAPEMODEL_H_

#include <string>
#include <vector>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ROI;

class Shape
{
 public:
  virtual ~Shape();

  virtual void setMargin( unsigned int margin )                      = 0;
  virtual bool isClose(unsigned int in_roi_x, unsigned int in_roi_y) = 0;

};

class ShapeModel
{
public:
  virtual   ~ShapeModel();
  virtual   std::string	           getName(void)	const	                  = 0;
  virtual   int		           parseImage( unsigned char* buffer, ROI *roi )  = 0;
  virtual   int		           getShapeCount(void) const			  = 0;
  virtual   Shape*	           getShape(int id) const			  = 0;
  virtual   Shape*	           getMostLikelyShape(void) const		  = 0;
};

} // end namespace firevision

#endif
