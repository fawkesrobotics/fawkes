
/***************************************************************************
 *  object.h - Fawkes BlackBoard Interface - ObjectPositionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007-2008  Tim Niemueller
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
  static const unsigned int TYPE_OTHER;
  static const unsigned int TYPE_BALL;
  static const unsigned int TYPE_OPPONENT;
  static const unsigned int TYPE_TEAMMEMBER;
  static const unsigned int TYPE_LINE;
  static const unsigned int TYPE_SELF;
  static const unsigned int FLAG_NONE;
  static const unsigned int FLAG_HAS_WORLD;
  static const unsigned int FLAG_HAS_RELATIVE_CARTESIAN;
  static const unsigned int FLAG_HAS_RELATIVE_POLAR;
  static const unsigned int FLAG_HAS_EULER_ANGLES;
  static const unsigned int FLAG_HAS_EXTENT;
  static const unsigned int FLAG_HAS_VOLUME_EXTENT;
  static const unsigned int FLAG_HAS_CIRCULAR_EXTENT;
  static const unsigned int FLAG_HAS_COVARIANCES;
  static const unsigned int FLAG_HAS_WORLD_VELOCITY;
  static const unsigned int FLAG_HAS_POLAR_VELOCITY;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int object_type; /**< 
      Object type, use constants to define
     */
    unsigned int flags; /**< 
      Bit-wise concatenated fields of FLAG_* constants. Denotes features that the
      writer of this interfaces provides. Use a bit-wise OR to concatenate multiple
      flags, use a bit-wise AND to check if a flag has been set.
     */
    float roll; /**< 
      Roll value for the orientation of the object in space.
     */
    float pitch; /**< 
      Pitch value for the orientation of the object in space.
     */
    float yaw; /**< 
      Yaw value for the orientation of the object in space.
     */
    float bearing; /**< 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
     */
    float slope; /**< 
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
    float extent_x; /**< 
      Extent of the seen object given in the relative x cartesian coordinate in m.
     */
    float extent_y; /**< 
      Extent of the seen object given in the relative y cartesian coordinate in m.
     */
    float extent_z; /**< 
      Extent of the seen object given in the relative z cartesian coordinate in m.
     */
    float bearing_velocity; /**< 
      Gives the velocity of the object for bearing in radians per second.
     */
    float slope_velocity; /**< 
      Gives the velocity of the object for slope in radians per second.
     */
    float distance_velocity; /**< 
      Gives the velocity of the object distance meter per second.
     */
    float dbs_velocity_covariance[9]; /**< 
      Covariance of Distance/Bearing/Slope velocity values. This is a 3x3 matrix ordered
      line by line, first three values represent row, next tree values second row and
      last three values last row from left to right each.
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
    bool visible; /**< 
      True, if object is visible.
     */
  } ObjectPositionInterface_data_t;

  ObjectPositionInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  ObjectPositionInterface();
  ~ObjectPositionInterface();

 public:
  virtual Message * create_message(const char *type) const;

  /* Methods */
  unsigned int object_type();
  void set_object_type(const unsigned int new_object_type);
  size_t maxlenof_object_type() const;
  unsigned int flags();
  void set_flags(const unsigned int new_flags);
  size_t maxlenof_flags() const;
  bool is_visible();
  void set_visible(const bool new_visible);
  size_t maxlenof_visible() const;
  float roll();
  void set_roll(const float new_roll);
  size_t maxlenof_roll() const;
  float pitch();
  void set_pitch(const float new_pitch);
  size_t maxlenof_pitch() const;
  float yaw();
  void set_yaw(const float new_yaw);
  size_t maxlenof_yaw() const;
  float bearing();
  void set_bearing(const float new_bearing);
  size_t maxlenof_bearing() const;
  float slope();
  void set_slope(const float new_slope);
  size_t maxlenof_slope() const;
  float distance();
  void set_distance(const float new_distance);
  size_t maxlenof_distance() const;
  float * dyp_covariance();
  void set_dyp_covariance(const float * new_dyp_covariance);
  size_t maxlenof_dyp_covariance() const;
  float world_x();
  void set_world_x(const float new_world_x);
  size_t maxlenof_world_x() const;
  float world_y();
  void set_world_y(const float new_world_y);
  size_t maxlenof_world_y() const;
  float world_z();
  void set_world_z(const float new_world_z);
  size_t maxlenof_world_z() const;
  float relative_x();
  void set_relative_x(const float new_relative_x);
  size_t maxlenof_relative_x() const;
  float relative_y();
  void set_relative_y(const float new_relative_y);
  size_t maxlenof_relative_y() const;
  float relative_z();
  void set_relative_z(const float new_relative_z);
  size_t maxlenof_relative_z() const;
  float * xyz_covariance();
  void set_xyz_covariance(const float * new_xyz_covariance);
  size_t maxlenof_xyz_covariance() const;
  float extent_x();
  void set_extent_x(const float new_extent_x);
  size_t maxlenof_extent_x() const;
  float extent_y();
  void set_extent_y(const float new_extent_y);
  size_t maxlenof_extent_y() const;
  float extent_z();
  void set_extent_z(const float new_extent_z);
  size_t maxlenof_extent_z() const;
  float bearing_velocity();
  void set_bearing_velocity(const float new_bearing_velocity);
  size_t maxlenof_bearing_velocity() const;
  float slope_velocity();
  void set_slope_velocity(const float new_slope_velocity);
  size_t maxlenof_slope_velocity() const;
  float distance_velocity();
  void set_distance_velocity(const float new_distance_velocity);
  size_t maxlenof_distance_velocity() const;
  float * dbs_velocity_covariance();
  void set_dbs_velocity_covariance(const float * new_dbs_velocity_covariance);
  size_t maxlenof_dbs_velocity_covariance() const;
  float world_x_velocity();
  void set_world_x_velocity(const float new_world_x_velocity);
  size_t maxlenof_world_x_velocity() const;
  float world_y_velocity();
  void set_world_y_velocity(const float new_world_y_velocity);
  size_t maxlenof_world_y_velocity() const;
  float world_z_velocity();
  void set_world_z_velocity(const float new_world_z_velocity);
  size_t maxlenof_world_z_velocity() const;
  float * xyz_velocity_covariance();
  void set_xyz_velocity_covariance(const float * new_xyz_velocity_covariance);
  size_t maxlenof_xyz_velocity_covariance() const;

};

#endif
