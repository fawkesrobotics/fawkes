
/***************************************************************************
 *  ObjectPositionInterface.h - Fawkes BlackBoard Interface - ObjectPositionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007-2008  Tim Niemueller
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

#ifndef __INTERFACES_OBJECTPOSITIONINTERFACE_H_
#define __INTERFACES_OBJECTPOSITIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class ObjectPositionInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(ObjectPositionInterface)
 /// @endcond
 public:
  /* constants */
  static const uint32_t TYPE_OTHER;
  static const uint32_t TYPE_BALL;
  static const uint32_t TYPE_OPPONENT;
  static const uint32_t TYPE_TEAMMEMBER;
  static const uint32_t TYPE_LINE;
  static const uint32_t TYPE_SELF;
  static const uint32_t TYPE_GOAL_BLUE;
  static const uint32_t TYPE_GOAL_YELLOW;
  static const uint32_t FLAG_NONE;
  static const uint32_t FLAG_HAS_WORLD;
  static const uint32_t FLAG_HAS_RELATIVE_CARTESIAN;
  static const uint32_t FLAG_HAS_RELATIVE_POLAR;
  static const uint32_t FLAG_HAS_EULER_ANGLES;
  static const uint32_t FLAG_HAS_EXTENT;
  static const uint32_t FLAG_HAS_VOLUME_EXTENT;
  static const uint32_t FLAG_HAS_CIRCULAR_EXTENT;
  static const uint32_t FLAG_HAS_COVARIANCES;
  static const uint32_t FLAG_HAS_WORLD_VELOCITY;
  static const uint32_t FLAG_HAS_Z_AS_ORI;
  static const uint32_t FLAG_IS_FIXED_OBJECT;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t object_type; /**< 
      Object type, use constants to define
     */
    uint32_t flags; /**< 
      Bit-wise concatenated fields of FLAG_* constants. Denotes features that the
      writer of this interfaces provides. Use a bit-wise OR to concatenate multiple
      flags, use a bit-wise AND to check if a flag has been set.
     */
    bool visible; /**< True, if object is visible. */
    bool valid; /**< True, if this position is valid. */
    int32_t visibility_history; /**< 
      The visibilitiy history indicates the number of consecutive positive or negative
      sightings. If the history is negative, there have been as many negative sightings
      (object not visible) as the absolute value of the history. A positive value denotes
      as many positive sightings. 0 shall only be used during the initialisation of the
      interface or if the visibility history is not filled.
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
    float distance; /**< 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
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
    float dbs_covariance[9]; /**< 
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
    float world_xyz_covariance[9]; /**< 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
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
    float relative_xyz_covariance[9]; /**< 
      Covariance of relative x/y/z values. This is a 3x3 matrix ordered line by line,
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
    float world_x_velocity; /**< 
      Velocity of object in the world coordinate system in X-direction in meter per second.
     */
    float world_y_velocity; /**< 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
     */
    float world_z_velocity; /**< 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
     */
    float world_xyz_velocity_covariance[9]; /**< 
      Covariance of WorldX/WorldY/WorldZ velocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
     */
    float relative_x_velocity; /**< 
      Velocity of object in the world coordinate system in X-direction in meter per second.
     */
    float relative_y_velocity; /**< 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
     */
    float relative_z_velocity; /**< 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
     */
    float relative_xyz_velocity_covariance[9]; /**< 
      Covariance of relative x/y/z velocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
     */
  } ObjectPositionInterface_data_t;
#pragma pack(pop)

  ObjectPositionInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  ObjectPositionInterface();
  ~ObjectPositionInterface();

 public:
  /* Methods */
  uint32_t object_type() const;
  void set_object_type(const uint32_t new_object_type);
  size_t maxlenof_object_type() const;
  uint32_t flags() const;
  void set_flags(const uint32_t new_flags);
  size_t maxlenof_flags() const;
  bool is_visible() const;
  void set_visible(const bool new_visible);
  size_t maxlenof_visible() const;
  bool is_valid() const;
  void set_valid(const bool new_valid);
  size_t maxlenof_valid() const;
  int32_t visibility_history() const;
  void set_visibility_history(const int32_t new_visibility_history);
  size_t maxlenof_visibility_history() const;
  float roll() const;
  void set_roll(const float new_roll);
  size_t maxlenof_roll() const;
  float pitch() const;
  void set_pitch(const float new_pitch);
  size_t maxlenof_pitch() const;
  float yaw() const;
  void set_yaw(const float new_yaw);
  size_t maxlenof_yaw() const;
  float distance() const;
  void set_distance(const float new_distance);
  size_t maxlenof_distance() const;
  float bearing() const;
  void set_bearing(const float new_bearing);
  size_t maxlenof_bearing() const;
  float slope() const;
  void set_slope(const float new_slope);
  size_t maxlenof_slope() const;
  float * dbs_covariance() const;
  float dbs_covariance(unsigned int index) const;
  void set_dbs_covariance(unsigned int index, const float new_dbs_covariance);
  void set_dbs_covariance(const float * new_dbs_covariance);
  size_t maxlenof_dbs_covariance() const;
  float world_x() const;
  void set_world_x(const float new_world_x);
  size_t maxlenof_world_x() const;
  float world_y() const;
  void set_world_y(const float new_world_y);
  size_t maxlenof_world_y() const;
  float world_z() const;
  void set_world_z(const float new_world_z);
  size_t maxlenof_world_z() const;
  float * world_xyz_covariance() const;
  float world_xyz_covariance(unsigned int index) const;
  void set_world_xyz_covariance(unsigned int index, const float new_world_xyz_covariance);
  void set_world_xyz_covariance(const float * new_world_xyz_covariance);
  size_t maxlenof_world_xyz_covariance() const;
  float relative_x() const;
  void set_relative_x(const float new_relative_x);
  size_t maxlenof_relative_x() const;
  float relative_y() const;
  void set_relative_y(const float new_relative_y);
  size_t maxlenof_relative_y() const;
  float relative_z() const;
  void set_relative_z(const float new_relative_z);
  size_t maxlenof_relative_z() const;
  float * relative_xyz_covariance() const;
  float relative_xyz_covariance(unsigned int index) const;
  void set_relative_xyz_covariance(unsigned int index, const float new_relative_xyz_covariance);
  void set_relative_xyz_covariance(const float * new_relative_xyz_covariance);
  size_t maxlenof_relative_xyz_covariance() const;
  float extent_x() const;
  void set_extent_x(const float new_extent_x);
  size_t maxlenof_extent_x() const;
  float extent_y() const;
  void set_extent_y(const float new_extent_y);
  size_t maxlenof_extent_y() const;
  float extent_z() const;
  void set_extent_z(const float new_extent_z);
  size_t maxlenof_extent_z() const;
  float world_x_velocity() const;
  void set_world_x_velocity(const float new_world_x_velocity);
  size_t maxlenof_world_x_velocity() const;
  float world_y_velocity() const;
  void set_world_y_velocity(const float new_world_y_velocity);
  size_t maxlenof_world_y_velocity() const;
  float world_z_velocity() const;
  void set_world_z_velocity(const float new_world_z_velocity);
  size_t maxlenof_world_z_velocity() const;
  float * world_xyz_velocity_covariance() const;
  float world_xyz_velocity_covariance(unsigned int index) const;
  void set_world_xyz_velocity_covariance(unsigned int index, const float new_world_xyz_velocity_covariance);
  void set_world_xyz_velocity_covariance(const float * new_world_xyz_velocity_covariance);
  size_t maxlenof_world_xyz_velocity_covariance() const;
  float relative_x_velocity() const;
  void set_relative_x_velocity(const float new_relative_x_velocity);
  size_t maxlenof_relative_x_velocity() const;
  float relative_y_velocity() const;
  void set_relative_y_velocity(const float new_relative_y_velocity);
  size_t maxlenof_relative_y_velocity() const;
  float relative_z_velocity() const;
  void set_relative_z_velocity(const float new_relative_z_velocity);
  size_t maxlenof_relative_z_velocity() const;
  float * relative_xyz_velocity_covariance() const;
  float relative_xyz_velocity_covariance(unsigned int index) const;
  void set_relative_xyz_velocity_covariance(unsigned int index, const float new_relative_xyz_velocity_covariance);
  void set_relative_xyz_velocity_covariance(const float * new_relative_xyz_velocity_covariance);
  size_t maxlenof_relative_xyz_velocity_covariance() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
