/***************************************************************************
 *  fit_accum.cpp - Implementation of 'fitted circle' accumulator
 *                  used by Fix-Point RCD Algorithm
 *
 *  Generated: Sat Sep 10 2005 17:28:12
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

#include <fvmodels/shape/accumulators/fit_accum.h>

#include <fvmodels/shape/circle.h>
#include <cmath>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

const float FitAccum::TOO_SMALL_DELTA = 1.0e-3f;

/** @class FitAccum <fvmodels/shape/accumulators/fit_accum.h>
 * FIT Accumulator.
 */

/** Constructor. */
FitAccum::FitAccum(void)
{
  reset();
}

/** Destructor. */
FitAccum::~FitAccum(void)
{
}

/** Reset. */
void
FitAccum::reset(void)
{
  count = 0;
  A00 = A01 = A02 = 0.0f;
  A10 = A11 = A12 = 0.0f;
  A20 = A21 = A22 = 0.0f;
  b0  = b1  = b2  = 0.0f;
}

/** Add point.
 * @param pt point
 */
void
FitAccum::addPoint(const upoint_t& pt)
{
  ++count;

  A00 += 4 * pt.x * pt.x;
  A01 += 4 * pt.x * pt.y;
  A02 += 2 * pt.x;

  A10 += 4 * pt.y * pt.x;
  A11 += 4 * pt.y * pt.y;
  A12 += 2 * pt.y;

  A20 += 2 * pt.x;
  A21 += 2 * pt.y;
  A22 += 1;

  float r2 = pt.x * pt.x + pt.y * pt.y;
  b0  += 2 * r2 * pt.x;
  b1  += 2 * r2 * pt.y;
  b2  += r2;
}


/** Remove point.
 * @param pt point
 */
void
FitAccum::removePoint(const upoint_t& pt)
{
  --count;
  A00 -= 4 * pt.x * pt.x;
  A01 -= 4 * pt.x * pt.y;
  A02 -= 2 * pt.x;

  A10 -= 4 * pt.y * pt.x;
  A11 -= 4 * pt.y * pt.y;
  A12 -= 2 * pt.y;

  A20 -= 2 * pt.x;
  A21 -= 2 * pt.y;
  A22 -= 1;

  float r2 = pt.x * pt.x + pt.y * pt.y;
  b0  -= 2 * r2 * pt.x;
  b1  -= 2 * r2 * pt.y;
  b2  -= r2;
}

/** Get count.
 * @return count
 */
int
FitAccum::getCount(void) const
{
  return count;
}

/** Get circle.
 * @return circle
 */
Circle*
FitAccum::getCircle(void) const
{
        // solve the resulting 3 by 3 equations
  static Circle c;

  float delta =   + A00 * A11 * A22 + A01 * A12 * A20 + A02 * A10 * A21
        - A00 * A12 * A21 - A01 * A10 * A22 - A02 * A11 * A20;

  if (delta > -TOO_SMALL_DELTA && delta < TOO_SMALL_DELTA)
  {
// printf("A=\n");
// printf("\t%f\t%f\t%f\n", A00, A01, A02);
// printf("\t%f\t%f\t%f\n", A10, A11, A12);
// printf("\t%f\t%f\t%f\n", A20, A21, A22);
// printf("b=\n");
// printf("\t%f\t%f\t%f\n", b0, b1, b2);
// printf("Delta too small: %e\n", delta);
    return NULL;
  }
  else
  {
    c.center.x = (float)( ( + b0  * A11 * A22 + A01 * A12 * b2  + A02 * b1  * A21
          - b0  * A12 * A21 - A01 * b1  * A22 - A02 * A11 * b2  ) / delta);
    c.center.y = (float)( ( + A00 * b1  * A22 + b0  * A12 * A20 + A02 * A10 * b2
          - A00 * A12 * b2  - b0  * A10 * A22 - A02 * b1  * A20 ) / delta);
    c.radius = (float)sqrt((+ A00 * A11 * b2  + A01 * b1  * A20 + b0  * A10 * A21
          - A00 * b1  * A21 - A01 * A10 * b2  - b0  * A11 * A20 ) / delta
          + c.center.x * c.center.x + c.center.y * c.center.y);
    c.count = count;
    return &c;
  }
}

} // end namespace firevision
