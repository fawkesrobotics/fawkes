/***************************************************************************
 *  fit_accum.h - Header for 'fitted circle' accumulator
 *                used by Fix-Point RCD Algorithm
 *
 *  Created: Sat Sep 10 17:25:55 2005
 *  Copyright  2005  Hu Yuxiao <Yuxiao.Hu@rwth-aachen.de>
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

#ifndef __FIREVISION_MODELS_SHAPE_ACCUMULATORS_FIT_ACCUM_H_
#define __FIREVISION_MODELS_SHAPE_ACCUMULATORS_FIT_ACCUM_H_

#include <utils/math/types.h>
#include <fvutils/base/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Circle;

class FitAccum
{
private:
  static const float TOO_SMALL_DELTA;

private:
  int count;
  float A00, A01, A02;
  float A10, A11, A12;
  float A20, A21, A22;

  float  b0,  b1,  b2;

public:
  FitAccum(void);
  ~FitAccum(void);

  void reset(void);
  void addPoint(const fawkes::upoint_t&);    // add a point
  void removePoint(const fawkes::upoint_t&); // remove a point

  int getCount(void) const;
  Circle* getCircle(void) const;
};

} // end namespace firevision

#endif
