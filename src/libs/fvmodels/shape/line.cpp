
/***************************************************************************
 *  line.cpp - Implementation of a line shape finder
 *
 *  Created: Thu May 16 00:00:00 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *                   Martin Heracles <Martin.Heracles@rwth-aachen.de>
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

#include <utils/math/angle.h>
#include <fvmodels/shape/line.h>

using namespace std;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LineShape <fvmodels/shape/line.h>
 * Line shape.
 */

/** Constructor.
 * @param roi_width ROI width
 * @param roi_height ROI height
 */
LineShape::LineShape(unsigned int roi_width, unsigned int roi_height)
{
  r = 0;
  phi = 0;
  count = 0;

  max_length = (int)sqrt( roi_width * roi_width + roi_height * roi_height );
  last_calc_r = last_calc_phi = 0.f;

  this->roi_width = roi_width;
  this->roi_height = roi_height;

}


/** Destructor. */
LineShape::~LineShape()
{
}


/** Print line.
 * @param stream stream to print to
 */
void
LineShape::printToStream(std::ostream &stream)
{
  stream << "r=" << r << "  phi=" << phi
	 << "  count= " << count;
}

void
LineShape::setMargin(unsigned int margin)
{
  this->margin = margin;
}


bool
LineShape::isClose(unsigned int in_roi_x, unsigned int in_roi_y)
{
  return false;
  /*
  Point p1(x1, y1);
  Point p2(x2, y2);

  Line  cl(p1, p2);

  Point p(x, y);
  /
  cout << "LineShape     (" << x1 << ", " << y1 << ") <-> (" << x2 << ", " << y2 << ")" << endl
       << "Point    (" << x  << ", " << y  << ")" << endl
       << "Distance: " << cl.getDistanceTo(p) << endl;
  /
  return (cl.GetDistanceTo(p) <= margin);
  */
}

/** Calc points for line. */
void
LineShape::calcPoints()
{

  if ((last_calc_r == r) && (last_calc_phi == phi)) return;
  last_calc_r   = r;
  last_calc_phi = phi;

  float rad_angle = fawkes::deg2rad(phi);

  // if true, point (x1, y1) will be moved the opposite direction
  bool reverse_direction = false;

  /* compute two points on the line.
     (x1, y1) will be somewhere inside or outside of ROI,
     (x2, y2) will be on a ROI edge (or on a prolongation thereof) */
  if ( rad_angle < M_PI/4 ) {
    x1 = (int)round( r * cos( rad_angle ) );
    y1 = (int)round( r * sin( rad_angle ) );
    y2 = 0;
    x2 = (int)round( r / cos( rad_angle ) );
  } else if ( rad_angle < M_PI/2 ) {
    x1 = (int)round( r * cos( rad_angle ) );
    y1 = (int)round( r * sin( rad_angle ) );
    x2 = 0;
    y2 = (int)round( r / cos( M_PI/2 - rad_angle ) );
  } else if ( rad_angle < 3.0/4.0 * M_PI ) {
    x1 = (int)round(-r * cos( M_PI - rad_angle ) );
    y1 = (int)round( r * sin( M_PI - rad_angle ) );
    x2 = 0;
    y2 = (int)round( r / cos( rad_angle - M_PI/2 ) );

    // the direction in which (x1, y1) has to be moved
    // depends on the sign of r
    if (r >= 0.0) {
      reverse_direction = true;
    }
  } else {
    // rad_angle <= M_PI
    x1 = (int)round(-r * cos( M_PI - rad_angle ) );
    y1 = (int)round( r * sin( M_PI - rad_angle ) );
    y2 = 0;
    x2 = (int)round(-r / cos( M_PI - rad_angle ) );
    
    // the direction in which (x1, y1) has to be moved
    // depends on the sign of r 
    if (r < 0.0) {
      reverse_direction = true;
    }
  }
  
  if ( ! (x1 == x2 &&
	  y1 == y2   ) ) {
    // move (x1, y1) away from (x2, y2) 
    // such that line (x1, y1)<->(x2, y2) spans the whole ROI
    float vx, vy, length;
    vx = x1 - x2 ;
    vy = y1 - y2 ;
    length = sqrt( vx * vx + vy * vy );
    
    vx /= length;
    vy /= length;
    vx *= max_length;
    vy *= max_length;

    if ( ! reverse_direction) {
      x1 += (int)vx;
      y1 += (int)vy;
    } else {
      x1 -= (int)vx;
      y1 -= (int)vy;
    }

  } else {
    // special case: both points are identical, hence could not be moved apart
    // (re-define first point "by hand")
    if (x2 == 0) {
      x1 = roi_width;
      y1 = y2;
    } else if (y2 == 0) {
      x1 = x2;
      y1 = roi_height;
    } else {
      cout << "ERROR!" << endl
	   << "  This case should not have occurred. Please have a look at method" << endl
	   << "  \"LineShape::calc()\". Treatment of special case is not correct." << endl;
      //exit(-1);
    }
  }
}


/** Get two points that define the line.
 * @param x1 contains x coordinate of first point upon return
 * @param y1 contains y coordinate of first point upon return
 * @param x2 contains x coordinate of second point upon return
 * @param y2 contains y coordinate of second point upon return
 */
void
LineShape::getPoints(int *x1, int *y1, int *x2, int *y2)
{
  calcPoints();

  *x1 = this->x1;
  *y1 = this->y1;
  *x2 = this->x2;
  *y2 = this->y2;
}

} // end namespace firevision
