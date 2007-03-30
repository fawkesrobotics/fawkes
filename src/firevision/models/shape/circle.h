
/***************************************************************************
 *  circle.h - Header of circle shape model
 *
 *  Generated: Thu May 16 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *                   Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#ifndef __FIREVISION_MODELS_SHAPE_CIRCLE_H_
#define __FIREVISION_MODELS_SHAPE_CIRCLE_H_

#include <vector>
#include <iostream>

#include <fvutils/types.h>
#include <fvutils/roi.h>
#include <models/shape/shapemodel.h>

// constants of the limits of the detected ball
const unsigned int TBY_CIRCLE_RADIUS_MAX = 600;
const unsigned int TBY_CIRCLE_RADIUS_MIN = 2;

class Circle : public Shape
{
 public:
  Circle();
  Circle(const center_in_roi_t& c, float r, int n = 0);

  void printToStream(std::ostream &stream);

  void setMargin( unsigned int margin );
  bool isClose( unsigned int in_roi_x, unsigned int in_roi_y );

  void fitCircle(std::vector< point_t >& points);

 public:
  /** Center of object in ROI */
  center_in_roi_t	center;
  /** Radius of object */
  float			radius;
  /** Number of pixels */
  int			count;
  /** Margin around shape */
  unsigned int          margin;

};

#endif // __FIREVISION_MODELS_SHAPE_CIRCLE_H_
