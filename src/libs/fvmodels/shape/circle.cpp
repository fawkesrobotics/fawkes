
/***************************************************************************
 *  circle.cpp - Implementation of a circle shape finder
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

#include <cmath>
#include <fvmodels/shape/circle.h>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Circle <fvmodels/shape/circle.h>
 * Circle shape.
 */

/** Constructor. */
Circle::Circle()
{
      center.x=center.y=0.0f;
      radius = -1.0f;
      count = 0;
}

/** Constructor.
 * @param c center
 * @param r radius
 * @param n number of pixels
 */
Circle::Circle(const center_in_roi_t& c, float r, int n)
{
      center = c;
      radius = r;
      count = n;
}

/** Print info.
 * @param stream stream to print to
 */
void
Circle::printToStream(std::ostream &stream)
{
        stream  << "center=(" << center.x << "," << center.y << ")"
                << "   radius=" << radius << "    count= " << count;
}

/** Fit circle.
 * Fit a circle through the given points.
 * @param points points to fit circle through.
 */
void
Circle::fitCircle (vector< upoint_t > &points)
{
        // due to fixed width, do not use arrays to save addressing time...
        double A00=0.0, A01=0.0, A02=0.0;
        double A10=0.0, A11=0.0, A12=0.0;
        double A20=0.0, A21=0.0, A22=0.0;
        double b0 =0.0, b1 =0.0, b2 =0.0;

        // generating A'A and A'b
        int count = points.size();
        for (int i = 0; i < count; i++)
        {
                upoint_t &t = points[i];
                double x0 = 2.0f * t.x;
                double y0 = 2.0f * t.y;
                double b = (double)(t.x * t.x + t.y * t.y);
                A00 += x0 * x0;
                A01 += x0 * y0;
                A02 += x0;
                A10 += y0 * x0;
                A11 += y0 * y0;
                A12 += y0;
                A20 += x0;
                A21 += y0;
                A22 += 1.0;
                b0  += x0 * b;
                b1  += y0 * b;
                b2  += b;
        }

        // solve the resulting 3 by 3 equations
        double delta =          + A00 * A11 * A22 + A01 * A12 * A20 + A02 * A10 * A21
                                - A00 * A12 * A21 - A01 * A10 * A22 - A02 * A11 * A20;
        center.x = (float)( (   + b0 * A11 * A22 + A01 * A12 * b2 + A02 * b1 * A21
                                - b0 * A12 * A21 - A01 * b1 * A22 - A02 * A11 * b2 ) / delta);
        center.y = (float)( (   + A00 * b1 * A22 + b0 * A12 * A20 + A02 * A10 * b2
                                - A00 * A12 * b2 - b0 * A10 * A22 - A02 * b1 * A20 ) / delta);
        radius = (float)sqrt( ( + A00 * A11 * b2 + A01 * b1 * A20 + b0 * A10 * A21
                                - A00 * b1 * A21 - A01 * A10 * b2 - b0 * A11 * A20 ) / delta
                                + center.x * center.x + center.y * center.y);
        count = points.size();
}


void
Circle::setMargin( unsigned int margin )
{
  this->margin = margin;
}


bool
Circle::isClose( unsigned int in_roi_x, unsigned int in_roi_y )
{
  float dx = in_roi_x - center.x;
  float dy = in_roi_y - center.y;

  float dist = sqrt( dx * dx + dy * dy );

  return ( (dist <= (radius + margin)) &&
           (dist >= (radius - margin)) );

}

} // end namespace firevision
