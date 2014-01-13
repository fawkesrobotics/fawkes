
/***************************************************************************
 *  rht_circle.h - Header of circle shape model
 *                 using Randomized Hough Transform
 *
 *  Created: Tue Jun 28 00:00:00 2005
 *  Copyright  2005  Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#ifndef __FIREVISION_RHT_CIRCLE_H_
#define __FIREVISION_RHT_CIRCLE_H_

#include <vector>
#include <iostream>

#include <utils/math/types.h>
#include <fvutils/base/types.h>
#include <fvmodels/shape/circle.h>
#include <fvmodels/shape/accumulators/ht_accum.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ROI;

class RhtCircleModel: public ShapeModel
{
 private:
  std::vector<Circle> m_Circles;
  RhtAccumulator accumulator;
  static const float RHT_MIN_RADIUS;
  static const float RHT_MAX_RADIUS;

 public:
  RhtCircleModel(void);
  virtual ~RhtCircleModel(void);

  std::string   getName(void) const {return std::string("RhtCircleModel");}
  int           parseImage(unsigned char* buffer, ROI *roi);
  int           getShapeCount(void) const;
  Circle*       getShape(int id) const;
  Circle*       getMostLikelyShape(void) const;

 private:
  void          calcCircle(     // for calculating circles from 3 points
                           const fawkes::upoint_t& p1,
                           const fawkes::upoint_t& p2,
                           const fawkes::upoint_t& p3,
                           center_in_roi_t& center,
                           float& radius);
};

} // end namespace firevision

#endif // __FIREVISION_RHT_CIRCLE_H_

