
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
/** Get ObjectType value.
 * 
      Object type, use constants to define
    
 * @return ObjectType value
 */
unsigned int
ObjectPositionInterface::getObjectType()
{
  return data->ObjectType;
}

/** Set ObjectType value.
 * 
      Object type, use constants to define
    
 * @param newObjectType new ObjectType value
 */
void
ObjectPositionInterface::setObjectType(unsigned int newObjectType)
{
  data->ObjectType = newObjectType;
}

/** Get Yaw value.
 * 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
    
 * @return Yaw value
 */
float
ObjectPositionInterface::getYaw()
{
  return data->Yaw;
}

/** Set Yaw value.
 * 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
    
 * @param newYaw new Yaw value
 */
void
ObjectPositionInterface::setYaw(float newYaw)
{
  data->Yaw = newYaw;
}

/** Get Pitch value.
 * 
      Angle between the robot's center position on the ground plane and the middle point
      of the object (e.g. this denotes the height of the object combined with the distance.
      The angle is given in radians.
    
 * @return Pitch value
 */
float
ObjectPositionInterface::getPitch()
{
  return data->Pitch;
}

/** Set Pitch value.
 * 
      Angle between the robot's center position on the ground plane and the middle point
      of the object (e.g. this denotes the height of the object combined with the distance.
      The angle is given in radians.
    
 * @param newPitch new Pitch value
 */
void
ObjectPositionInterface::setPitch(float newPitch)
{
  data->Pitch = newPitch;
}

/** Get Distance value.
 * 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
    
 * @return Distance value
 */
float
ObjectPositionInterface::getDistance()
{
  return data->Distance;
}

/** Set Distance value.
 * 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
    
 * @param newDistance new Distance value
 */
void
ObjectPositionInterface::setDistance(float newDistance)
{
  data->Distance = newDistance;
}

/** Get DYPCovariance value.
 * 
      Covariance of Distance/Yaw/Pitch values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @return DYPCovariance value
 */
float *
ObjectPositionInterface::getDYPCovariance()
{
  return data->DYPCovariance;
}

/** Set DYPCovariance value.
 * 
      Covariance of Distance/Yaw/Pitch values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @param newDYPCovariance new DYPCovariance value
 */
void
ObjectPositionInterface::setDYPCovariance(float * newDYPCovariance)
{
  memcpy(data->DYPCovariance, newDYPCovariance, sizeof(float) * 9);
}

/** Get WorldX value.
 * 
      This is the X coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @return WorldX value
 */
int
ObjectPositionInterface::getWorldX()
{
  return data->WorldX;
}

/** Set WorldX value.
 * 
      This is the X coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @param newWorldX new WorldX value
 */
void
ObjectPositionInterface::setWorldX(int newWorldX)
{
  data->WorldX = newWorldX;
}

/** Get WorldY value.
 * 
      This is the Y coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right and Z pointing downwards.
    
 * @return WorldY value
 */
int
ObjectPositionInterface::getWorldY()
{
  return data->WorldY;
}

/** Set WorldY value.
 * 
      This is the Y coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right and Z pointing downwards.
    
 * @param newWorldY new WorldY value
 */
void
ObjectPositionInterface::setWorldY(int newWorldY)
{
  data->WorldY = newWorldY;
}

/** Get WorldZ value.
 * 
      This is the Z coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @return WorldZ value
 */
int
ObjectPositionInterface::getWorldZ()
{
  return data->WorldZ;
}

/** Set WorldZ value.
 * 
      This is the Z coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @param newWorldZ new WorldZ value
 */
void
ObjectPositionInterface::setWorldZ(int newWorldZ)
{
  data->WorldZ = newWorldZ;
}

/** Get XYZCovariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @return XYZCovariance value
 */
float *
ObjectPositionInterface::getXYZCovariance()
{
  return data->XYZCovariance;
}

/** Set XYZCovariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @param newXYZCovariance new XYZCovariance value
 */
void
ObjectPositionInterface::setXYZCovariance(float * newXYZCovariance)
{
  memcpy(data->XYZCovariance, newXYZCovariance, sizeof(float) * 9);
}

/** Get YawVelocity value.
 * 
      Gives the velocity of the object for yaw in radians per second.
    
 * @return YawVelocity value
 */
float
ObjectPositionInterface::getYawVelocity()
{
  return data->YawVelocity;
}

/** Set YawVelocity value.
 * 
      Gives the velocity of the object for yaw in radians per second.
    
 * @param newYawVelocity new YawVelocity value
 */
void
ObjectPositionInterface::setYawVelocity(float newYawVelocity)
{
  data->YawVelocity = newYawVelocity;
}

/** Get PitchVelocity value.
 * 
      Gives the velocity of the object for pitch in radians per second.
    
 * @return PitchVelocity value
 */
float
ObjectPositionInterface::getPitchVelocity()
{
  return data->PitchVelocity;
}

/** Set PitchVelocity value.
 * 
      Gives the velocity of the object for pitch in radians per second.
    
 * @param newPitchVelocity new PitchVelocity value
 */
void
ObjectPositionInterface::setPitchVelocity(float newPitchVelocity)
{
  data->PitchVelocity = newPitchVelocity;
}

/** Get DistanceVelocity value.
 * 
      Gives the velocity of the object distance meter per second.
    
 * @return DistanceVelocity value
 */
float
ObjectPositionInterface::getDistanceVelocity()
{
  return data->DistanceVelocity;
}

/** Set DistanceVelocity value.
 * 
      Gives the velocity of the object distance meter per second.
    
 * @param newDistanceVelocity new DistanceVelocity value
 */
void
ObjectPositionInterface::setDistanceVelocity(float newDistanceVelocity)
{
  data->DistanceVelocity = newDistanceVelocity;
}

/** Get DYPVelocityCovariance value.
 * 
      Covariance of Distance/Yaw/Pitch velocityvalues. This is a 3x3 matrix ordered line by
      line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @return DYPVelocityCovariance value
 */
float *
ObjectPositionInterface::getDYPVelocityCovariance()
{
  return data->DYPVelocityCovariance;
}

/** Set DYPVelocityCovariance value.
 * 
      Covariance of Distance/Yaw/Pitch velocityvalues. This is a 3x3 matrix ordered line by
      line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @param newDYPVelocityCovariance new DYPVelocityCovariance value
 */
void
ObjectPositionInterface::setDYPVelocityCovariance(float * newDYPVelocityCovariance)
{
  memcpy(data->DYPVelocityCovariance, newDYPVelocityCovariance, sizeof(float) * 9);
}

/** Get WorldXVelocity value.
 * 
      Velocity of object in the world coordinate system in X-direction in meter per second.
    
 * @return WorldXVelocity value
 */
int
ObjectPositionInterface::getWorldXVelocity()
{
  return data->WorldXVelocity;
}

/** Set WorldXVelocity value.
 * 
      Velocity of object in the world coordinate system in X-direction in meter per second.
    
 * @param newWorldXVelocity new WorldXVelocity value
 */
void
ObjectPositionInterface::setWorldXVelocity(int newWorldXVelocity)
{
  data->WorldXVelocity = newWorldXVelocity;
}

/** Get WorldYVelocity value.
 * 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
    
 * @return WorldYVelocity value
 */
int
ObjectPositionInterface::getWorldYVelocity()
{
  return data->WorldYVelocity;
}

/** Set WorldYVelocity value.
 * 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
    
 * @param newWorldYVelocity new WorldYVelocity value
 */
void
ObjectPositionInterface::setWorldYVelocity(int newWorldYVelocity)
{
  data->WorldYVelocity = newWorldYVelocity;
}

/** Get WorldZVelocity value.
 * 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
    
 * @return WorldZVelocity value
 */
int
ObjectPositionInterface::getWorldZVelocity()
{
  return data->WorldZVelocity;
}

/** Set WorldZVelocity value.
 * 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
    
 * @param newWorldZVelocity new WorldZVelocity value
 */
void
ObjectPositionInterface::setWorldZVelocity(int newWorldZVelocity)
{
  data->WorldZVelocity = newWorldZVelocity;
}

/** Get XYZVelocityCovariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ valocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @return XYZVelocityCovariance value
 */
float *
ObjectPositionInterface::getXYZVelocityCovariance()
{
  return data->XYZVelocityCovariance;
}

/** Set XYZVelocityCovariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ valocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @param newXYZVelocityCovariance new XYZVelocityCovariance value
 */
void
ObjectPositionInterface::setXYZVelocityCovariance(float * newXYZVelocityCovariance)
{
  memcpy(data->XYZVelocityCovariance, newXYZVelocityCovariance, sizeof(float) * 9);
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

