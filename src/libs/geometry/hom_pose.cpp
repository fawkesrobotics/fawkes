
/***************************************************************************
 *  hom_pose.cpp - Homogenous Pose
 *
 *  Created: Sun April 13 17:52:43 2008
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

#include <geometry/hom_pose.h>

/** @class HomPose <geometry/hom_pose.h>
 * A homogeneous pose combines a position with an orienation in space.
 * @author Daniel Beck
 */

/** Constructor.
 * Constructs a two-dimensional pose.
 * @param x the x-coordinate of the position
 * @param y the y-coordinate of the position
 * @param yaw the orienations in the xy-plane
 */
HomPose::HomPose(float x, float y, float yaw)
  : HomPoint(x, y)
{
  m_roll  = 0.0;
  m_pitch = 0.0;
  m_yaw   = yaw;
}

/** Constructor.
 * Constructs a three-dimensional pose.
 * @param x the x-coordinate of the position
 * @param y the y-coordinate of the position
 * @param z the z-coordinate of the position
 * @param roll the orienations in the yz-plane
 * @param pitch the orienations in the xz-plane
 * @param yaw the orienations in the xy-plane
 */
HomPose::HomPose(float x, float y, float z, float roll, float pitch, float yaw)
  : HomPoint(x, y, z)
{
  m_roll  = roll;
  m_pitch = pitch;
  m_yaw   = yaw;
}

/** Copy constructor.
 * @parm h a homoegeneous coordinate
 */
HomPose::HomPose(const HomCoord& h)
  : HomPoint(h)
{
  m_roll  = 0.0;
  m_pitch = 0.0;
  m_yaw   = 0.0;
}

/** Destructor. */
HomPose::~HomPose()
{
}

/** RO-getter for roll.
 * @return the value
 */
float
HomPose::roll() const
{
  return m_roll;
}

/** RW-getter for roll.
 * @return a reference to the roll variable
 */
float&
HomPose::roll()
{
  return m_roll;
}

/** Setter function for roll.
 * @param roll the new roll value
 */
void
HomPose::roll(float roll)
{
  m_roll = roll;
}

/** RO-getter for pitch.
 * @return the value
 */
float
HomPose::pitch() const
{
  return m_pitch;
}

/** RW-getter for pitch.
 * @return a reference to the pitch variable
 */
float&
HomPose::pitch()
{
  return m_pitch;
}

/** Setter function for pitch.
 * @param pitch the new pitch value
 */
void
HomPose::pitch(float pitch)
{
  m_pitch = pitch;
}

/** RO-getter for yaw.
 * @return the value
 */
float
HomPose::yaw() const
{
  return m_yaw;
}

/** RW-getter for yaw.
 * @return a reference to the yaw variable
 */
float&
HomPose::yaw()
{
  return m_yaw;
}

/** Setter function for yaw.
 * @param yaw the new yaw value
 */
void
HomPose::yaw(float yaw)
{
  m_yaw = yaw;
}

/** Get the positional part of the pose.
 * @return the position
 */
HomPoint
HomPose::pos() const
{
  HomPoint pos;
  pos.x() = x();
  pos.y() = y();
  pos.z() = z();

  return pos;
}

HomPose&
HomPose::rotate_x(float rad)
{
  HomCoord::rotate_x(rad);
  m_roll += rad;

  return *this;
}

HomPose&
HomPose::rotate_y(float rad)
{
  HomCoord::rotate_y(rad);
  m_roll += rad;

  return *this;
}

HomPose&
HomPose::rotate_z(float rad)
{
  HomCoord::rotate_z(rad);
  m_roll += rad;

  return *this;
}
