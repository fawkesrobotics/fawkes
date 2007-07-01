
/***************************************************************************
 *  object.h - Fawkes BlackBoard Interface - ObjectPositionInterface
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

#ifndef __INTERFACES_OBJECT_H_
#define __INTERFACES_OBJECT_H_

#include <interface/interface.h>
#include <interface/message.h>

class ObjectPositionInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(ObjectPositionInterface)
 /// @endcond
 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int ObjectType; /**< 
      Object type, use constants to define
     */
    int WorldX; /**< 
      This is the X coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
     */
    int WorldY; /**< 
      This is the Y coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right and Z pointing downwards.
     */
    int WorldZ; /**< 
      This is the Z coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
     */
    int WorldXVelocity; /**< 
      Velocity of object in the world coordinate system in X-direction in meter per second.
     */
    int WorldYVelocity; /**< 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
     */
    int WorldZVelocity; /**< 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
     */
    float Yaw; /**< 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
     */
    float Pitch; /**< 
      Angle between the robot's center position on the ground plane and the middle point
      of the object (e.g. this denotes the height of the object combined with the distance.
      The angle is given in radians.
     */
    float Distance; /**< 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
     */
    float DYPCovariance[9]; /**< 
      Covariance of Distance/Yaw/Pitch values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
     */
    float XYZCovariance[9]; /**< 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
     */
    float YawVelocity; /**< 
      Gives the velocity of the object for yaw in radians per second.
     */
    float PitchVelocity; /**< 
      Gives the velocity of the object for pitch in radians per second.
     */
    float DistanceVelocity; /**< 
      Gives the velocity of the object distance meter per second.
     */
    float DYPVelocityCovariance[9]; /**< 
      Covariance of Distance/Yaw/Pitch velocityvalues. This is a 3x3 matrix ordered line by
      line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
     */
    float XYZVelocityCovariance[9]; /**< 
      Covariance of WorldX/WorldY/WorldZ valocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
     */
  } ObjectPositionInterface_data_t;

  ObjectPositionInterface_data_t *data;

 public:
  /* constants */
  static const unsigned int OTHER;
  static const unsigned int BALL;
  static const unsigned int OPPONENT;
  static const unsigned int TEAMMEMBER;

  /* messages */
  virtual bool messageValid(const Message *message) const;
 private:
  ObjectPositionInterface();
  ~ObjectPositionInterface();

 public:
  /* Methods */
  unsigned int getObjectType();
  void setObjectType(unsigned int newObjectType);
  float getYaw();
  void setYaw(float newYaw);
  float getPitch();
  void setPitch(float newPitch);
  float getDistance();
  void setDistance(float newDistance);
  float * getDYPCovariance();
  void setDYPCovariance(float * newDYPCovariance);
  int getWorldX();
  void setWorldX(int newWorldX);
  int getWorldY();
  void setWorldY(int newWorldY);
  int getWorldZ();
  void setWorldZ(int newWorldZ);
  float * getXYZCovariance();
  void setXYZCovariance(float * newXYZCovariance);
  float getYawVelocity();
  void setYawVelocity(float newYawVelocity);
  float getPitchVelocity();
  void setPitchVelocity(float newPitchVelocity);
  float getDistanceVelocity();
  void setDistanceVelocity(float newDistanceVelocity);
  float * getDYPVelocityCovariance();
  void setDYPVelocityCovariance(float * newDYPVelocityCovariance);
  int getWorldXVelocity();
  void setWorldXVelocity(int newWorldXVelocity);
  int getWorldYVelocity();
  void setWorldYVelocity(int newWorldYVelocity);
  int getWorldZVelocity();
  void setWorldZVelocity(int newWorldZVelocity);
  float * getXYZVelocityCovariance();
  void setXYZVelocityCovariance(float * newXYZVelocityCovariance);

};

#endif
