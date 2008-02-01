
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
 * Foobar
 */



/** Constructor */
NavigatorInterface::NavigatorInterface() : Interface()
{
  data_size = sizeof(NavigatorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavigatorInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
}
/** Destructor */
NavigatorInterface::~NavigatorInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get foo value.
 * Foo
 * @return foo value
 */
int
NavigatorInterface::foo()
{
  return data->foo;
}

/** Set foo value.
 * Foo
 * @param new_foo new foo value
 */
void
NavigatorInterface::set_foo(const int new_foo)
{
  data->foo = new_foo;
}

/* =========== messages =========== */
/** @class NavigatorInterface::TargetMessage interfaces/navigator.h
 * TargetMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_orientation initial value for orientation
 */
NavigatorInterface::TargetMessage::TargetMessage(float ini_x, float ini_y, float ini_orientation) : Message("TargetMessage")
{
  data_size = sizeof(TargetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TargetMessage_data_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->orientation = ini_orientation;
}
/** Constructor */
NavigatorInterface::TargetMessage::TargetMessage() : Message("TargetMessage")
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
/** Get x value.
 * X-coordinate of the target, in the robot's coordinate system.
 * @return x value
 */
float
NavigatorInterface::TargetMessage::x()
{
  return data->x;
}

/** Set x value.
 * X-coordinate of the target, in the robot's coordinate system.
 * @param new_x new x value
 */
void
NavigatorInterface::TargetMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of the target, in the robot's coordinate system.
 * @return y value
 */
float
NavigatorInterface::TargetMessage::y()
{
  return data->y;
}

/** Set y value.
 * Y-coordinate of the target, in the robot's coordinate system.
 * @param new_y new y value
 */
void
NavigatorInterface::TargetMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get orientation value.
 * The orientation of the robot at the target.
 * @return orientation value
 */
float
NavigatorInterface::TargetMessage::orientation()
{
  return data->orientation;
}

/** Set orientation value.
 * The orientation of the robot at the target.
 * @param new_orientation new orientation value
 */
void
NavigatorInterface::TargetMessage::set_orientation(const float new_orientation)
{
  data->orientation = new_orientation;
}

/** @class NavigatorInterface::MaxVelocityMessage interfaces/navigator.h
 * MaxVelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_velocity initial value for velocity
 */
NavigatorInterface::MaxVelocityMessage::MaxVelocityMessage(float ini_velocity) : Message("MaxVelocityMessage")
{
  data_size = sizeof(MaxVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MaxVelocityMessage_data_t *)data_ptr;
  data->velocity = ini_velocity;
}
/** Constructor */
NavigatorInterface::MaxVelocityMessage::MaxVelocityMessage() : Message("MaxVelocityMessage")
{
  data_size = sizeof(MaxVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MaxVelocityMessage_data_t *)data_ptr;
}
/** Destructor */
NavigatorInterface::MaxVelocityMessage::~MaxVelocityMessage()
{
}
/* Methods */
/** Get velocity value.
 * Maximum velocity of the robot.
 * @return velocity value
 */
float
NavigatorInterface::MaxVelocityMessage::velocity()
{
  return data->velocity;
}

/** Set velocity value.
 * Maximum velocity of the robot.
 * @param new_velocity new velocity value
 */
void
NavigatorInterface::MaxVelocityMessage::set_velocity(const float new_velocity)
{
  data->velocity = new_velocity;
}

/** @class NavigatorInterface::ObstacleMessage interfaces/navigator.h
 * ObstacleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 * @param ini_width initial value for width
 */
NavigatorInterface::ObstacleMessage::ObstacleMessage(float ini_x, float ini_y, float ini_width) : Message("ObstacleMessage")
{
  data_size = sizeof(ObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ObstacleMessage_data_t *)data_ptr;
  data->x = ini_x;
  data->y = ini_y;
  data->width = ini_width;
}
/** Constructor */
NavigatorInterface::ObstacleMessage::ObstacleMessage() : Message("ObstacleMessage")
{
  data_size = sizeof(ObstacleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ObstacleMessage_data_t *)data_ptr;
}
/** Destructor */
NavigatorInterface::ObstacleMessage::~ObstacleMessage()
{
}
/* Methods */
/** Get x value.
 * X-coordinate of the obstacle.
 * @return x value
 */
float
NavigatorInterface::ObstacleMessage::x()
{
  return data->x;
}

/** Set x value.
 * X-coordinate of the obstacle.
 * @param new_x new x value
 */
void
NavigatorInterface::ObstacleMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of the obstacle.
 * @return y value
 */
float
NavigatorInterface::ObstacleMessage::y()
{
  return data->y;
}

/** Set y value.
 * Y-coordinate of the obstacle.
 * @param new_y new y value
 */
void
NavigatorInterface::ObstacleMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** Get width value.
 * Width of the obstacle.
 * @return width value
 */
float
NavigatorInterface::ObstacleMessage::width()
{
  return data->width;
}

/** Set width value.
 * Width of the obstacle.
 * @param new_width new width value
 */
void
NavigatorInterface::ObstacleMessage::set_width(const float new_width)
{
  data->width = new_width;
}

/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
NavigatorInterface::message_valid(const Message *message) const
{
  const TargetMessage *m0 = dynamic_cast<const TargetMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const MaxVelocityMessage *m1 = dynamic_cast<const MaxVelocityMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const ObstacleMessage *m2 = dynamic_cast<const ObstacleMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavigatorInterface)
/// @endcond

