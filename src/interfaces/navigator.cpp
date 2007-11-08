
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
 * @param ini_distance initial value for distance
 * @param ini_angle initial value for angle
 * @param ini_x initial value for x
 * @param ini_y initial value for y
 */
NavigatorInterface::TargetMessage::TargetMessage(float ini_distance, float ini_angle, float ini_x, float ini_y) : Message()
{
  data_size = sizeof(TargetMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TargetMessage_data_t *)data_ptr;
  data->distance = ini_distance;
  data->angle = ini_angle;
  data->x = ini_x;
  data->y = ini_y;
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
/** Get distance value.
 * Distance to the target.
 * @return distance value
 */
float
NavigatorInterface::TargetMessage::distance()
{
  return data->distance;
}

/** Set distance value.
 * Distance to the target.
 * @param new_distance new distance value
 */
void
NavigatorInterface::TargetMessage::set_distance(const float new_distance)
{
  data->distance = new_distance;
}

/** Get angle value.
 * Angle of the target.
 * @return angle value
 */
float
NavigatorInterface::TargetMessage::angle()
{
  return data->angle;
}

/** Set angle value.
 * Angle of the target.
 * @param new_angle new angle value
 */
void
NavigatorInterface::TargetMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Get x value.
 * X-coordinate of the target.
 * @return x value
 */
float
NavigatorInterface::TargetMessage::x()
{
  return data->x;
}

/** Set x value.
 * X-coordinate of the target.
 * @param new_x new x value
 */
void
NavigatorInterface::TargetMessage::set_x(const float new_x)
{
  data->x = new_x;
}

/** Get y value.
 * Y-coordinate of the target.
 * @return y value
 */
float
NavigatorInterface::TargetMessage::y()
{
  return data->y;
}

/** Set y value.
 * Y-coordinate of the target.
 * @param new_y new y value
 */
void
NavigatorInterface::TargetMessage::set_y(const float new_y)
{
  data->y = new_y;
}

/** @class NavigatorInterface::VelocityMessage interfaces/navigator.h
 * VelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_velocity initial value for velocity
 */
NavigatorInterface::VelocityMessage::VelocityMessage(float ini_velocity) : Message()
{
  data_size = sizeof(VelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (VelocityMessage_data_t *)data_ptr;
  data->velocity = ini_velocity;
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
/** Get velocity value.
 * Velocity of the robot.
 * @return velocity value
 */
float
NavigatorInterface::VelocityMessage::velocity()
{
  return data->velocity;
}

/** Set velocity value.
 * Velocity of the robot.
 * @param new_velocity new velocity value
 */
void
NavigatorInterface::VelocityMessage::set_velocity(const float new_velocity)
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
NavigatorInterface::ObstacleMessage::ObstacleMessage(float ini_x, float ini_y, float ini_width) : Message()
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
NavigatorInterface::ObstacleMessage::ObstacleMessage() : Message()
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
  const ObstacleMessage *m2 = dynamic_cast<const ObstacleMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavigatorInterface)
/// @endcond

