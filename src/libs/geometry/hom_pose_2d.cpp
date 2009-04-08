
/***************************************************************************
 *  hom_pose_2d.cpp - 2-dimensional Homogenous Pose
 *
 *  Created: Fri Oct 10 11:13:32 2008
 *  Copyright  2008  Daniel Beck
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

#include <geometry/hom_pose_2d.h>
#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>
#include <cmath>

/** @class fawkes::HomPose2d <geometry/hom_pose_2d.h>
 * A 2-dimensional pose, i.e. a combination of position and
 * orientation.
 * @author Daniel Beck
 */

namespace fawkes {

/** Constructor.
 * @param position the posititon
 * @param orientation the orientation
 */
HomPose2d::HomPose2d(const HomPoint& position, const HomVector& orientation)
{
  m_position  = new HomPoint(position);
  m_orientation = new HomVector(orientation);

  m_orientation->unit();
  m_yaw = atan2f( m_orientation->y(), m_orientation->x() );

  register_primitives();
}

/** Constructor.
 * @param x the x-coordinate of the position
 * @param y the y-coordinate of the position
 * @param yaw the angle of the orientation wrt. the current frame
 */
HomPose2d::HomPose2d(float x, float y, float yaw)
{
  m_position  = new HomPoint(x, y);
  m_orientation = new HomVector(1.0, 0.0);

  m_orientation->rotate_z(yaw);
  m_yaw = yaw;

  register_primitives();
}

/** Copy constructor.
 * @param p the other pose
 */
HomPose2d::HomPose2d(const HomPose2d& p)
{
  m_position  = new HomPoint( *p.m_position );
  m_orientation = new HomVector( *p.m_orientation );

  m_yaw = p.m_yaw;

  register_primitives();
}

/** Destructor. */
HomPose2d::~HomPose2d()
{
  delete m_position;
  delete m_orientation;
}

/** Assignment operator.
 * @param p the rhs pose
 * @return reference to the assigned pose
 */
const HomPose2d&
HomPose2d::operator=(const HomPose2d& p)
{
  (*m_position)  = (*p.m_position);
  (*m_orientation) = (*p.m_orientation);

  m_yaw = p.m_yaw;

  return *this;
}

/** Get the x-coordinate of the position.
 * @return the x-coordinate of the position.
 */
float
HomPose2d::x() const
{
  return m_position->x();
}

/** Set the x-coordinate of the position.
 * @param x the new x-coordinate of the position.
 */
void
HomPose2d::x(float x)
{
  m_position->x(x);
}

/** Get the y-coordinate of the position.
 * @return the y-coordinate of the position.
 */
float
HomPose2d::y() const
{
  return m_position->y();
}

/** Set the y-coordinate of the position.
 * @param y the new x-coordinate of the position.
 */
void
HomPose2d::y(float y)
{
  m_position->y(y);
}

/** Get the angle of the current orientation [0...2pi].
 * @return the angle of the current orientation
 */
float
HomPose2d::yaw() const
{
  return m_yaw;
}

/** Set the angle of the orientation.
 * @param yaw the new angle of the orientation
 */
void
HomPose2d::yaw(float yaw)
{
  if ( yaw < 0 ||
       yaw > 2 * M_PI )
  {
    m_yaw = yaw - 2 * M_PI * floorf( yaw / ( 2 * M_PI ) );
  }
  else
  { m_yaw = yaw; }

  delete m_orientation;
  m_orientation = new HomVector(1.0, 0.0);
  m_orientation->rotate_z(m_yaw);
}

/** Get the position.
 * @return the position
 */
const HomPoint&
HomPose2d::position() const
{
  return *m_position;
}

/** Set the positional part of the pose.
 * @param p the new position
 */
void
HomPose2d::set_position(const HomPoint& p)
{
  *m_position = p;
}

/** Get the orientation vector.
 * @return the orientation vector
 */
const HomVector&
HomPose2d::orientation() const
{
  return *m_orientation;
}

void
HomPose2d::register_primitives()
{
  add_primitive( m_position );
  add_primitive( m_orientation );
}

void
HomPose2d::post_transform()
{
  m_yaw = atan2f( m_orientation->y(), m_orientation->x() );

  if ( m_yaw < 0 ||
       m_yaw > 2 * M_PI )
  {
    m_yaw = m_yaw - 2 * M_PI * floorf( m_yaw / ( 2 * M_PI ) );
  }
}

} // end namespace fawkes
