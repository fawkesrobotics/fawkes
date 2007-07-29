
/***************************************************************************
 *  navigator.cpp - Fawkes BlackBoard Interface - NavigatorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#include <interfaces/navigator.h>

#include <string.h>
#include <stdlib.h>

/** @class NavigatorInterface interfaces/navigator.h
 * NavigatorInterface Fawkes BlackBoard Interface.
 * Fooba
 */



/** Constructor */
NavigatorInterface::NavigatorInterface() : Interface()
{
  data_size = sizeof(NavigatorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavigatorInterface_data_t *)data_ptr;
}
/** Destructor */
NavigatorInterface::~NavigatorInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get Foo value.
 * Foo
 * @return Foo value
 */
int
NavigatorInterface::getFoo()
{
  return data->Foo;
}

/** Set Foo value.
 * Foo
 * @param newFoo new Foo value
 */
void
NavigatorInterface::setFoo(const int newFoo)
{
  data->Foo = newFoo;
}

/* =========== messages =========== */
/** @class NavigatorInterface::TargetMessage interfaces/navigator.h
 * TargetMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniDistance initial value for Distance
 * @param iniAngle initial value for Angle
 * @param iniX initial value for X
 * @param iniY initial value for Y
 */
NavigatorInterface::TargetMessage::TargetMessage(float iniDistance, float iniAngle, float iniX, float iniY) : Message()
{
  data_size = sizeof(TargetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TargetMessage_data_t *)data_ptr;
  data->Distance = iniDistance;
  data->Angle = iniAngle;
  data->X = iniX;
  data->Y = iniY;
}
/** Constructor */
NavigatorInterface::TargetMessage::TargetMessage() : Message()
{
  data_size = sizeof(TargetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TargetMessage_data_t *)data_ptr;
}
/** Destructor */
NavigatorInterface::TargetMessage::~TargetMessage()
{
}
/* Methods */
/** Get Distance value.
 * Distance to the target.
 * @return Distance value
 */
float
NavigatorInterface::TargetMessage::getDistance()
{
  return data->Distance;
}

/** Set Distance value.
 * Distance to the target.
 * @param newDistance new Distance value
 */
void
NavigatorInterface::TargetMessage::setDistance(const float newDistance)
{
  data->Distance = newDistance;
}

/** Get Angle value.
 * Angle of the target.
 * @return Angle value
 */
float
NavigatorInterface::TargetMessage::getAngle()
{
  return data->Angle;
}

/** Set Angle value.
 * Angle of the target.
 * @param newAngle new Angle value
 */
void
NavigatorInterface::TargetMessage::setAngle(const float newAngle)
{
  data->Angle = newAngle;
}

/** Get X value.
 * X-coordinate of the target.
 * @return X value
 */
float
NavigatorInterface::TargetMessage::getX()
{
  return data->X;
}

/** Set X value.
 * X-coordinate of the target.
 * @param newX new X value
 */
void
NavigatorInterface::TargetMessage::setX(const float newX)
{
  data->X = newX;
}

/** Get Y value.
 * Y-coordinate of the target.
 * @return Y value
 */
float
NavigatorInterface::TargetMessage::getY()
{
  return data->Y;
}

/** Set Y value.
 * Y-coordinate of the target.
 * @param newY new Y value
 */
void
NavigatorInterface::TargetMessage::setY(const float newY)
{
  data->Y = newY;
}

/** @class NavigatorInterface::VelocityMessage interfaces/navigator.h
 * VelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniVelocity initial value for Velocity
 */
NavigatorInterface::VelocityMessage::VelocityMessage(float iniVelocity) : Message()
{
  data_size = sizeof(VelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (VelocityMessage_data_t *)data_ptr;
  data->Velocity = iniVelocity;
}
/** Constructor */
NavigatorInterface::VelocityMessage::VelocityMessage() : Message()
{
  data_size = sizeof(VelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (VelocityMessage_data_t *)data_ptr;
}
/** Destructor */
NavigatorInterface::VelocityMessage::~VelocityMessage()
{
}
/* Methods */
/** Get Velocity value.
 * Velocity of the robot.
 * @return Velocity value
 */
float
NavigatorInterface::VelocityMessage::getVelocity()
{
  return data->Velocity;
}

/** Set Velocity value.
 * Velocity of the robot.
 * @param newVelocity new Velocity value
 */
void
NavigatorInterface::VelocityMessage::setVelocity(const float newVelocity)
{
  data->Velocity = newVelocity;
}

/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
NavigatorInterface::messageValid(const Message *message) const
{
  const TargetMessage *m0 = dynamic_cast<const TargetMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const VelocityMessage *m1 = dynamic_cast<const VelocityMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavigatorInterface)
/// @endcond

