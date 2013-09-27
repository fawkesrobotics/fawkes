/***************************************************************************
 *  fc_accum.cpp - Implementation of 'fitted circle' accumulator
 *                 used by Randomized Stable Circle Fitting Algorithm
 *
 *  Generated: Fri Sep 09 2005 22:50:12
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

#include <fvmodels/shape/accumulators/fc_accum.h>

#include <cmath>
#include <cstdio>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

const float FittedCircle::TOO_SMALL_DELTA = 1.0e-3f;

/** @class FittedCircle <fvmodels/shape/accumulators/fc_accum.h>
 * FittedCircle accumulator.
 */

/** Constructor. */
FittedCircle::FittedCircle(void)
{
  reset();
}

/** Destructor. */
FittedCircle::~FittedCircle(void)
{
}

/** Reset. */
void
FittedCircle::reset(void)
{
  count = 0;
  for (int i=0; i<2; ++i)
  {
    circle_matrices[i].A00 = circle_matrices[i].A01 = circle_matrices[i].A02 = 0.0f;
    circle_matrices[i].A10 = circle_matrices[i].A11 = circle_matrices[i].A12 = 0.0f;
    circle_matrices[i].A20 = circle_matrices[i].A21 = circle_matrices[i].A22 = 0.0f;
    circle_matrices[i].b0 = circle_matrices[i].b1 = circle_matrices[i].b2 = 0.0f;
  }
  current_circle = 0;
  point_added = false;
}


/** Add point.
 * @param pt point
 * @return distance from circle center
 */
float
FittedCircle::addPoint(const upoint_t& pt)
{
  int next_circle = 1 - current_circle;
  point_added = true;

  circle_matrices[next_circle].A00 += 4 * pt.x * pt.x;
  circle_matrices[next_circle].A01 += 4 * pt.x * pt.y;
  circle_matrices[next_circle].A02 += 2 * pt.x;

  circle_matrices[next_circle].A10 += 4 * pt.y * pt.x;
  circle_matrices[next_circle].A11 += 4 * pt.y * pt.y;
  circle_matrices[next_circle].A12 += 2 * pt.y;

  circle_matrices[next_circle].A20 += 2 * pt.x;
  circle_matrices[next_circle].A21 += 2 * pt.y;
  circle_matrices[next_circle].A22 += 1;

  float r2 = pt.x * pt.x + pt.y * pt.y;
  circle_matrices[next_circle].b0  += 2 * r2 * pt.x;
  circle_matrices[next_circle].b1  += 2 * r2 * pt.y;
  circle_matrices[next_circle].b2  += r2;

  float dist;

  Circle* p = fitCircle(&circle_matrices[next_circle]);
  if (p)
  {
    float dx = p->center.x - pt.x;
    float dy = p->center.y - pt.y;
    float r  = p->radius;
    dist = fabs(sqrt(dx*dx+dy*dy)-r);
  }
  else
  {
    dist = 0;
  }

  return dist;
}


/** Remove point.
 * @param pt point
 */
void
FittedCircle::removePoint(const upoint_t& pt)
{
  int next_circle = 1 - current_circle;
  point_added = true;

  circle_matrices[next_circle].A00 -= 4 * pt.x * pt.x;
  circle_matrices[next_circle].A01 -= 4 * pt.x * pt.y;
  circle_matrices[next_circle].A02 -= 2 * pt.x;

  circle_matrices[next_circle].A10 -= 4 * pt.y * pt.x;
  circle_matrices[next_circle].A11 -= 4 * pt.y * pt.y;
  circle_matrices[next_circle].A12 -= 2 * pt.y;

  circle_matrices[next_circle].A20 -= 2 * pt.x;
  circle_matrices[next_circle].A21 -= 2 * pt.y;
  circle_matrices[next_circle].A22 -= 1;

  float r2 = pt.x * pt.x + pt.y * pt.y;
  circle_matrices[next_circle].b0  -= 2 * r2 * pt.x;
  circle_matrices[next_circle].b1  -= 2 * r2 * pt.y;
  circle_matrices[next_circle].b2  -= r2;
}


/** Distance.
 * @param pt point
 * @param current current
 * @return distance
 */
float
FittedCircle::distanceTo(const upoint_t& pt, bool current)
{
  int id = current?current_circle:(1-current_circle);
  Circle* p = fitCircle(&circle_matrices[id]);
  if (p)
  {
    float dx = p->center.x - pt.x;
    float dy = p->center.y - pt.y;
    return fabs(sqrt(dx*dx+dy*dy)-p->radius);
  }
  else
  {
    // There is no circle, perhaps it is a line...
    return 100000.0f; // temporarily... should fit a line...
  }
}


/** Commit. */
void
FittedCircle::commit(void)
{
  if (point_added)
  {
    current_circle = 1 - current_circle;
    point_added = false;
    ++count;
  }
}

/** Get count.
 * @return count
 */
int
FittedCircle::getCount(void) const
{
  return count;
}


/** Get circle.
 * @return circle
 */
Circle*
FittedCircle::getCircle(void) const
{
  return fitCircle(const_cast<circle_matrix*>(&circle_matrices[current_circle]));
}


/** Fit circle.
 * @param p circle matrix
 * @return circle
 */
Circle*
FittedCircle::fitCircle(circle_matrix* p) const
{
        // solve the resulting 3 by 3 equations
  static Circle c;
  float delta =   + p->A00 * p->A11 * p->A22 + p->A01 * p->A12 * p->A20 + p->A02 * p->A10 * p->A21
        - p->A00 * p->A12 * p->A21 - p->A01 * p->A10 * p->A22 - p->A02 * p->A11 * p->A20;
  if (delta > -TOO_SMALL_DELTA && delta < TOO_SMALL_DELTA)
  {
printf("A=\n");
printf("\t%f\t%f\t%f\n", p->A00, p->A01, p->A02);
printf("\t%f\t%f\t%f\n", p->A10, p->A11, p->A12);
printf("\t%f\t%f\t%f\n", p->A20, p->A21, p->A22);
printf("b=\n");
printf("\t%f\t%f\t%f\n", p->b0, p->b1, p->b2);
printf("Delta too small: %e\n", delta);
    return NULL;
  }
  else
  {
    c.center.x = (float)( ( + p->b0  * p->A11 * p->A22 + p->A01 * p->A12 * p->b2  + p->A02 * p->b1  * p->A21
          - p->b0  * p->A12 * p->A21 - p->A01 * p->b1  * p->A22 - p->A02 * p->A11 * p->b2  ) / delta);
    c.center.y = (float)( ( + p->A00 * p->b1  * p->A22 + p->b0  * p->A12 * p->A20 + p->A02 * p->A10 * p->b2
          - p->A00 * p->A12 * p->b2  - p->b0  * p->A10 * p->A22 - p->A02 * p->b1  * p->A20 ) / delta);
    c.radius = (float)sqrt((+ p->A00 * p->A11 * p->b2  + p->A01 * p->b1  * p->A20 + p->b0  * p->A10 * p->A21
          - p->A00 * p->b1  * p->A21 - p->A01 * p->A10 * p->b2  - p->b0  * p->A11 * p->A20 ) / delta
          + c.center.x * c.center.x + c.center.y * c.center.y);
    c.count = count;
    return &c;
  }
}

} // end namespace firevision
