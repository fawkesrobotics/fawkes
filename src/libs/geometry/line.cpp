
/***************************************************************************
 *  line.cpp - A line
 *
 *  Created: Fri Sep 28 16:12:33 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
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

#include <geometry/line.h>
#include <utils/math/angle.h>
#include <math.h>
#include <iostream>


/** @class Line <geometry/line.h>
 * Well, what can one say about a straight line?
 */

/* @var Line::mBasePoint
 * A point that lies on the line (Aufpunkt)
 */

/* @var Line::mDirection
 * A vector pointing in the direction of the line
 */


/**Constructor.
 * @param p a point on the line ("Aufpunkt")
 * @param v a vector that lies on the line
 */
Line::Line(const HomPoint& p, const HomVector& v)
  : GeomObj(HomTransform().reset(), HomPoint(0.0, 0.0, 0.0))
{
  mBasePoint = p;
  mDirection = v;

  // align direction vector with x-axis of object's CS
  // project the direction vector in the xy-plane and calculate
  // the angle to the x-axis
  float z_angle = atan( mDirection.y() / mDirection.x() );
  // project the direction vector in the xy-plane and calculate
  // the angle between itself and the projected vector
  HomVector direction_xy = mDirection;
  direction_xy.z() = 0.0;
  float y_angle = -atan( mDirection.z() / direction_xy.length() );

  // first rotate around the z-axis ...
  mToRefCS.rotate_z(z_angle);
  // then around the y-axis
  mToRefCS.rotate_y(y_angle);

  mToRefCS.trans(mBasePoint.x(), mBasePoint.y(), mBasePoint.z());
  
  mChanged = true;
}


/**Constructor.
 * @param p1 one point that lies on the line
 * @param p2 another point that lies on the line
 */
Line::Line(const HomPoint& p1, const HomPoint& p2)
  : GeomObj(HomTransform().reset(), HomPoint(0.0, 0.0, 0.0))
{
  mBasePoint = p1;
  mDirection = p2 - p1;

  // align direction vector with x-axis of object's CS
  // project the direction vector in the xy-plane and calculate
  // the angle to the x-axis
  float z_angle = atan( mDirection.y() / mDirection.x() );
  // project the direction vector in the xy-plane and calculate
  // the angle between itself and the projected vector
  HomVector direction_xy = mDirection;
  direction_xy.z() = 0.0;
  float y_angle = -atan( mDirection.z() / direction_xy.length() );

  // first rotate around the z-axis ...
  mToRefCS.rotate_z(z_angle);
  // then around the y-axis
  mToRefCS.rotate_y(y_angle);

  mToRefCS.trans(mBasePoint.x(), mBasePoint.y(), mBasePoint.z());
  
  mChanged = true;
}


/**Destructor */
Line::~Line()
{
}


/**Apply a transformation to the line.
 * @param t transform
 * @return a reference to itself
 */
Line&
Line::apply_transform(const HomTransform& t)
{
  _apply_transform(t);

  return *this;
}


/**Apply a transformation to the line wrt. the reference CS.
 * @param t transform
 * @return a reference to itself
 */
Line&
Line::apply_transform_ref(const HomTransform& t)
{
  _apply_transform_ref(t);

  return *this;
}


/**Translate the object wrt. its local CS.
 * @param trans_x translation along the x-axis
 * @param trans_y translation along the y-axis
 * @param trans_z translation along the z-axis
 * @return a reference to *this
 */
Line&
Line::trans(float trans_x, float trans_y, float trans_z)
{
  _trans(trans_x, trans_y, trans_z);

  return *this;
}


/**Translate the object wrt. the reference CS.
 * @param trans_x translation along the x-axis
 * @param trans_y translation along the y-axis
 * @param trans_z translation along the z-axis
 * @return a reference to *this
 */
Line&
Line::trans_ref(float trans_x, float trans_y, float trans_z)
{
  _trans_ref(trans_x, trans_y, trans_z);

  return *this;
}


/**Rotate the object around the x-axis of its CS.
 * @param angle the angle
 * @return a reference to *this
 */
Line&
Line::rotate_x(float angle)
{
  _rotate_x(angle);

  return *this;
}


/**Rotate the object around the y-axis of its CS.
 * @param angle the angle
 * @return a reference to *this
 */
Line&
Line::rotate_y(float angle)
{
  _rotate_y(angle);

  return *this;
}


/**Rotate the object around the z-axis of its CS.
 * @param angle the angle
 * @return a reference to *this
 */
Line&
Line::rotate_z(float angle)
{
  _rotate_z(angle);

  return *this;
}


/**Rotate the object around the x-axis of the reference CS.
 * @param angle the angle
 * @return a reference to *this
 */
Line&
Line::rotate_x_ref(float angle)
{
  _rotate_x_ref(angle);

  return *this;
}


/**Rotate the object around the y-axis of the reference CS.
 * @param angle the angle
 * @return a reference to *this
 */
Line&
Line::rotate_y_ref(float angle)
{
  _rotate_y_ref(angle);

  return *this;
}


/**Rotate the object around the z-axis of the reference CS.
 * @param angle the angle
 * @return a reference to *this
 */
Line&
Line::rotate_z_ref(float angle)
{
  _rotate_z_ref(angle);

  return *this;
}
