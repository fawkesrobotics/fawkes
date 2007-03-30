
/***************************************************************************
 *  shapemodel.h - Abstract class defining a shape model
 *
 *  Generated: Tue May 03 19:50:02 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *             2005       Martin Heracles <Martin.Heracles@rwth-aachen.de>
 *             2005       Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_MODELS_SHAPE_SHAPEMODEL_H_
#define __FIREVISION_MODELS_SHAPE_SHAPEMODEL_H_

#include <string>
#include <vector>

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

#endif
