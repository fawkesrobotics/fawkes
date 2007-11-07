
/***************************************************************************
 *  object.cpp - Fawkes BlackBoard Interface - ObjectPositionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Tim Niemueller
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

#include <interfaces/object.h>

#include <string.h>
#include <stdlib.h>

/** @class ObjectPositionInterface interfaces/object.h
 * ObjectPositionInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to arbitrary object positions and velocities. You
      can use it to store the position of any object in the RoboCup domain. There is a type
      indicator for the RoboCup soccer domain to easily distinguish several well
      known objects. You may choose not to use this for other application in which case
      the value should be other (which is also the default).
    
 */


/** OTHER constant */
const unsigned int ObjectPositionInterface::OTHER = 0;
/** BALL constant */
const unsigned int ObjectPositionInterface::BALL = 1;
/** OPPONENT constant */
const unsigned int ObjectPositionInterface::OPPONENT = 2;
/** TEAMMEMBER constant */
const unsigned int ObjectPositionInterface::TEAMMEMBER = 3;

/** Constructor */
ObjectPositionInterface::ObjectPositionInterface() : Interface()
{
  data_size = sizeof(ObjectPositionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ObjectPositionInterface_data_t *)data_ptr;
}
/** Destructor */
ObjectPositionInterface::~ObjectPositionInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get object_type value.
 * 
      Object type, use constants to define
    
 * @return object_type value
 */
unsigned int
ObjectPositionInterface::object_type()
{
  return data->object_type;
}

/** Set object_type value.
 * 
      Object type, use constants to define
    
 * @param new_object_type new object_type value
 */
void
ObjectPositionInterface::set_object_type(const unsigned int new_object_type)
{
  data->object_type = new_object_type;
}

/** Get visible value.
 * 
      True, if object is visible.
    
 * @return visible value
 */
bool
ObjectPositionInterface::is_visible()
{
  return data->visible;
}

/** Set visible value.
 * 
      True, if object is visible.
    
 * @param new_visible new visible value
 */
void
ObjectPositionInterface::set_visible(const bool new_visible)
{
  data->visible = new_visible;
}

/** Get yaw value.
 * 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
    
 * @return yaw value
 */
float
ObjectPositionInterface::yaw()
{
  return data->yaw;
}

/** Set yaw value.
 * 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
    
 * @param new_yaw new yaw value
 */
void
ObjectPositionInterface::set_yaw(const float new_yaw)
{
  data->yaw = new_yaw;
}

/** Get pitch value.
 * 
      Angle between the robot's center position on the ground plane and the middle point
      of the object (e.g. this denotes the height of the object combined with the distance.
      The angle is given in radians.
    
 * @return pitch value
 */
float
ObjectPositionInterface::pitch()
{
  return data->pitch;
}

/** Set pitch value.
 * 
      Angle between the robot's center position on the ground plane and the middle point
      of the object (e.g. this denotes the height of the object combined with the distance.
      The angle is given in radians.
    
 * @param new_pitch new pitch value
 */
void
ObjectPositionInterface::set_pitch(const float new_pitch)
{
  data->pitch = new_pitch;
}

/** Get distance value.
 * 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
    
 * @return distance value
 */
float
ObjectPositionInterface::distance()
{
  return data->distance;
}

/** Set distance value.
 * 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
    
 * @param new_distance new distance value
 */
void
ObjectPositionInterface::set_distance(const float new_distance)
{
  data->distance = new_distance;
}

/** Get dyp_covariance value.
 * 
      Covariance of Distance/Yaw/Pitch values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @return dyp_covariance value
 */
float *
ObjectPositionInterface::dyp_covariance()
{
  return data->dyp_covariance;
}

/** Set dyp_covariance value.
 * 
      Covariance of Distance/Yaw/Pitch values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @param new_dyp_covariance new dyp_covariance value
 */
void
ObjectPositionInterface::set_dyp_covariance(const float * new_dyp_covariance)
{
  memcpy(data->dyp_covariance, new_dyp_covariance, sizeof(float) * 9);
}

/** Get world_x value.
 * 
      This is the X coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @return world_x value
 */
float
ObjectPositionInterface::world_x()
{
  return data->world_x;
}

/** Set world_x value.
 * 
      This is the X coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @param new_world_x new world_x value
 */
void
ObjectPositionInterface::set_world_x(const float new_world_x)
{
  data->world_x = new_world_x;
}

/** Get world_y value.
 * 
      This is the Y coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right and Z pointing downwards.
    
 * @return world_y value
 */
float
ObjectPositionInterface::world_y()
{
  return data->world_y;
}

/** Set world_y value.
 * 
      This is the Y coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right and Z pointing downwards.
    
 * @param new_world_y new world_y value
 */
void
ObjectPositionInterface::set_world_y(const float new_world_y)
{
  data->world_y = new_world_y;
}

/** Get world_z value.
 * 
      This is the Z coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @return world_z value
 */
float
ObjectPositionInterface::world_z()
{
  return data->world_z;
}

/** Set world_z value.
 * 
      This is the Z coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @param new_world_z new world_z value
 */
void
ObjectPositionInterface::set_world_z(const float new_world_z)
{
  data->world_z = new_world_z;
}

/** Get relative_x value.
 * 
      This is the X coordinate in the cartesian right-handed robot coordinate system.
    
 * @return relative_x value
 */
float
ObjectPositionInterface::relative_x()
{
  return data->relative_x;
}

/** Set relative_x value.
 * 
      This is the X coordinate in the cartesian right-handed robot coordinate system.
    
 * @param new_relative_x new relative_x value
 */
void
ObjectPositionInterface::set_relative_x(const float new_relative_x)
{
  data->relative_x = new_relative_x;
}

/** Get relative_y value.
 * 
      This is the Y coordinate in the cartesian right-handed robot coordinate system.
    
 * @return relative_y value
 */
float
ObjectPositionInterface::relative_y()
{
  return data->relative_y;
}

/** Set relative_y value.
 * 
      This is the Y coordinate in the cartesian right-handed robot coordinate system.
    
 * @param new_relative_y new relative_y value
 */
void
ObjectPositionInterface::set_relative_y(const float new_relative_y)
{
  data->relative_y = new_relative_y;
}

/** Get relative_z value.
 * 
      This is the Z coordinate in the cartesian right-handed robot coordinate system.
    
 * @return relative_z value
 */
float
ObjectPositionInterface::relative_z()
{
  return data->relative_z;
}

/** Set relative_z value.
 * 
      This is the Z coordinate in the cartesian right-handed robot coordinate system.
    
 * @param new_relative_z new relative_z value
 */
void
ObjectPositionInterface::set_relative_z(const float new_relative_z)
{
  data->relative_z = new_relative_z;
}

/** Get xyz_covariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @return xyz_covariance value
 */
float *
ObjectPositionInterface::xyz_covariance()
{
  return data->xyz_covariance;
}

/** Set xyz_covariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @param new_xyz_covariance new xyz_covariance value
 */
void
ObjectPositionInterface::set_xyz_covariance(const float * new_xyz_covariance)
{
  memcpy(data->xyz_covariance, new_xyz_covariance, sizeof(float) * 9);
}

/** Get yaw_velocity value.
 * 
      Gives the velocity of the object for yaw in radians per second.
    
 * @return yaw_velocity value
 */
float
ObjectPositionInterface::yaw_velocity()
{
  return data->yaw_velocity;
}

/** Set yaw_velocity value.
 * 
      Gives the velocity of the object for yaw in radians per second.
    
 * @param new_yaw_velocity new yaw_velocity value
 */
void
ObjectPositionInterface::set_yaw_velocity(const float new_yaw_velocity)
{
  data->yaw_velocity = new_yaw_velocity;
}

/** Get pitch_velocity value.
 * 
      Gives the velocity of the object for pitch in radians per second.
    
 * @return pitch_velocity value
 */
float
ObjectPositionInterface::pitch_velocity()
{
  return data->pitch_velocity;
}

/** Set pitch_velocity value.
 * 
      Gives the velocity of the object for pitch in radians per second.
    
 * @param new_pitch_velocity new pitch_velocity value
 */
void
ObjectPositionInterface::set_pitch_velocity(const float new_pitch_velocity)
{
  data->pitch_velocity = new_pitch_velocity;
}

/** Get distance_velocity value.
 * 
      Gives the velocity of the object distance meter per second.
    
 * @return distance_velocity value
 */
float
ObjectPositionInterface::distance_velocity()
{
  return data->distance_velocity;
}

/** Set distance_velocity value.
 * 
      Gives the velocity of the object distance meter per second.
    
 * @param new_distance_velocity new distance_velocity value
 */
void
ObjectPositionInterface::set_distance_velocity(const float new_distance_velocity)
{
  data->distance_velocity = new_distance_velocity;
}

/** Get dyp_velocity_covariance value.
 * 
      Covariance of Distance/Yaw/Pitch velocityvalues. This is a 3x3 matrix ordered line by
      line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @return dyp_velocity_covariance value
 */
float *
ObjectPositionInterface::dyp_velocity_covariance()
{
  return data->dyp_velocity_covariance;
}

/** Set dyp_velocity_covariance value.
 * 
      Covariance of Distance/Yaw/Pitch velocityvalues. This is a 3x3 matrix ordered line by
      line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @param new_dyp_velocity_covariance new dyp_velocity_covariance value
 */
void
ObjectPositionInterface::set_dyp_velocity_covariance(const float * new_dyp_velocity_covariance)
{
  memcpy(data->dyp_velocity_covariance, new_dyp_velocity_covariance, sizeof(float) * 9);
}

/** Get world_x_velocity value.
 * 
      Velocity of object in the world coordinate system in X-direction in meter per second.
    
 * @return world_x_velocity value
 */
float
ObjectPositionInterface::world_x_velocity()
{
  return data->world_x_velocity;
}

/** Set world_x_velocity value.
 * 
      Velocity of object in the world coordinate system in X-direction in meter per second.
    
 * @param new_world_x_velocity new world_x_velocity value
 */
void
ObjectPositionInterface::set_world_x_velocity(const float new_world_x_velocity)
{
  data->world_x_velocity = new_world_x_velocity;
}

/** Get world_y_velocity value.
 * 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
    
 * @return world_y_velocity value
 */
float
ObjectPositionInterface::world_y_velocity()
{
  return data->world_y_velocity;
}

/** Set world_y_velocity value.
 * 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
    
 * @param new_world_y_velocity new world_y_velocity value
 */
void
ObjectPositionInterface::set_world_y_velocity(const float new_world_y_velocity)
{
  data->world_y_velocity = new_world_y_velocity;
}

/** Get world_z_velocity value.
 * 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
    
 * @return world_z_velocity value
 */
float
ObjectPositionInterface::world_z_velocity()
{
  return data->world_z_velocity;
}

/** Set world_z_velocity value.
 * 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
    
 * @param new_world_z_velocity new world_z_velocity value
 */
void
ObjectPositionInterface::set_world_z_velocity(const float new_world_z_velocity)
{
  data->world_z_velocity = new_world_z_velocity;
}

/** Get xyz_velocity_covariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ valocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @return xyz_velocity_covariance value
 */
float *
ObjectPositionInterface::xyz_velocity_covariance()
{
  return data->xyz_velocity_covariance;
}

/** Set xyz_velocity_covariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ valocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @param new_xyz_velocity_covariance new xyz_velocity_covariance value
 */
void
ObjectPositionInterface::set_xyz_velocity_covariance(const float * new_xyz_velocity_covariance)
{
  memcpy(data->xyz_velocity_covariance, new_xyz_velocity_covariance, sizeof(float) * 9);
}

/* =========== messages =========== */
/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
ObjectPositionInterface::messageValid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ObjectPositionInterface)
/// @endcond

