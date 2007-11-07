
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
 public:
  /* constants */
  static const unsigned int OTHER;
  static const unsigned int BALL;
  static const unsigned int OPPONENT;
  static const unsigned int TEAMMEMBER;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    float yaw; /**< 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
     */
    float pitch; /**< 
      Angle between the robot's center position on the ground plane and the middle point
      of the object (e.g. this denotes the height of the object combined with the distance.
      The angle is given in radians.
     */
    float distance; /**< 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
     */
    float dyp_covariance[9]; /**< 
      Covariance of Distance/Yaw/Pitch values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
     */
    float world_x; /**< 
      This is the X coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
     */
    float world_y; /**< 
      This is the Y coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right and Z pointing downwards.
     */
    float world_z; /**< 
      This is the Z coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
     */
    float relative_x; /**< 
      This is the X coordinate in the cartesian right-handed robot coordinate system.
     */
    float relative_y; /**< 
      This is the Y coordinate in the cartesian right-handed robot coordinate system.
     */
    float relative_z; /**< 
      This is the Z coordinate in the cartesian right-handed robot coordinate system.
     */
    float xyz_covariance[9]; /**< 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
     */
    float yaw_velocity; /**< 
      Gives the velocity of the object for yaw in radians per second.
     */
    float pitch_velocity; /**< 
      Gives the velocity of the object for pitch in radians per second.
     */
    float distance_velocity; /**< 
      Gives the velocity of the object distance meter per second.
     */
    float dyp_velocity_covariance[9]; /**< 
      Covariance of Distance/Yaw/Pitch velocityvalues. This is a 3x3 matrix ordered line by
      line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
     */
    float world_x_velocity; /**< 
      Velocity of object in the world coordinate system in X-direction in meter per second.
     */
    float world_y_velocity; /**< 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
     */
    float world_z_velocity; /**< 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
     */
    float xyz_velocity_covariance[9]; /**< 
      Covariance of WorldX/WorldY/WorldZ valocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
     */
    unsigned int object_type : 16; /**< 
      Object type, use constants to define
     */
    unsigned int supports_relative : 1; /**< 
      Bit field, 1 if the current interface in general has support for relative position data.
     */
    unsigned int supports_global : 1; /**< 
      Bit field, 1 if the current interface in general has support for global (world) position data.
     */
    unsigned int has_relative : 1; /**< 
      Bit field, 1 if the current interface at the moment has valid relative position data.
     */
    unsigned int has_global : 1; /**< 
      Bit field, 1 if the current interface at the moment has valid global (world) position data.
     */
    bool visible; /**< 
      True, if object is visible.
     */
  } ObjectPositionInterface_data_t;

  ObjectPositionInterface_data_t *data;

 public:
  /* messages */
  virtual bool messageValid(const Message *message) const;
 private:
  ObjectPositionInterface();
  ~ObjectPositionInterface();

 public:
  /* Methods */
  unsigned int object_type();
  void set_object_type(const unsigned int new_object_type);
  unsigned int supports_relative();
  void set_supports_relative(const unsigned int new_supports_relative);
  unsigned int supports_global();
  void set_supports_global(const unsigned int new_supports_global);
  unsigned int has_relative();
  void set_has_relative(const unsigned int new_has_relative);
  unsigned int has_global();
  void set_has_global(const unsigned int new_has_global);
  bool is_visible();
  void set_visible(const bool new_visible);
  float yaw();
  void set_yaw(const float new_yaw);
  float pitch();
  void set_pitch(const float new_pitch);
  float distance();
  void set_distance(const float new_distance);
  float * dyp_covariance();
  void set_dyp_covariance(const float * new_dyp_covariance);
  float world_x();
  void set_world_x(const float new_world_x);
  float world_y();
  void set_world_y(const float new_world_y);
  float world_z();
  void set_world_z(const float new_world_z);
  float relative_x();
  void set_relative_x(const float new_relative_x);
  float relative_y();
  void set_relative_y(const float new_relative_y);
  float relative_z();
  void set_relative_z(const float new_relative_z);
  float * xyz_covariance();
  void set_xyz_covariance(const float * new_xyz_covariance);
  float yaw_velocity();
  void set_yaw_velocity(const float new_yaw_velocity);
  float pitch_velocity();
  void set_pitch_velocity(const float new_pitch_velocity);
  float distance_velocity();
  void set_distance_velocity(const float new_distance_velocity);
  float * dyp_velocity_covariance();
  void set_dyp_velocity_covariance(const float * new_dyp_velocity_covariance);
  float world_x_velocity();
  void set_world_x_velocity(const float new_world_x_velocity);
  float world_y_velocity();
  void set_world_y_velocity(const float new_world_y_velocity);
  float world_z_velocity();
  void set_world_z_velocity(const float new_world_z_velocity);
  float * xyz_velocity_covariance();
  void set_xyz_velocity_covariance(const float * new_xyz_velocity_covariance);

};

#endif
