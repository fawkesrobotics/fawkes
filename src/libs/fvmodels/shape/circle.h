
/***************************************************************************
 *  circle.h - Header of circle shape model
 *
 *  Created: Thu May 16 00:00:00 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *                   Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#ifndef __FIREVISION_MODELS_SHAPE_CIRCLE_H_
#define __FIREVISION_MODELS_SHAPE_CIRCLE_H_

#include <vector>
#include <iostream>

#include <utils/math/types.h>
#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>
#include <fvmodels/shape/shapemodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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

  void fitCircle(std::vector< fawkes::upoint_t >& points);

 public:
  /** Center of object in ROI */
  center_in_roi_t center;
  /** Radius of object */
  float     radius;
  /** Number of pixels */
  int     count;
  /** Margin around shape */
  unsigned int          margin;

};

} // end namespace firevision

#endif // __FIREVISION_MODELS_SHAPE_CIRCLE_H_
