/***************************************************************************
 *  types.h - Fawkes tf types (based on ROS tf)
 *
 *  Created: Tue Oct 18 17:03:47 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

/* This code is based on ROS tf with the following copyright and license:
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LIBS_TF_TYPES_H_
#define __LIBS_TF_TYPES_H_

#ifndef HAVE_TF
#  error HAVE_TF not defined, forgot CFLAGS_TF in Makefile or bullet no installed?
#endif

#include <utils/time/time.h>
#include <tf/exceptions.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

#include <string>
#include <cmath>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** Scalar datatype. */
typedef btScalar Scalar;
/** Representaton of orientation or rotation depending on context. */
typedef btQuaternion Quaternion;
/** Representation of a translation. */
typedef btVector3 Vector3;
/** Representation of a point (position). */
typedef btVector3 Point;
/** Representation of a translation and rotation. */
typedef btTransform Transform;
/** Representation of pose (position and orientation). */
typedef btTransform Pose;
/** Representation of 3x3 matrix. */
typedef btMatrix3x3 Matrix3x3;

/// Internally used to reference frames efficiently
typedef uint32_t CompactFrameID;

/** Transform that contains a timestamp and frame IDs. */
class StampedTransform : public Transform
{
 public:
  /// Timestamp of this transform.
  fawkes::Time stamp;
  /// Parent/reference frame ID.
  std::string frame_id;
  /// Frame ID of child frame, e.g. the transform denotes the
  /// transform from the parent frame to this child.
  std::string child_frame_id;

  /** Constructor.
   * @param input transform
   * @param timestamp timestamp for this transform
   * @param frame_id parent frame ID
   * @param child_frame_id child frame ID
   */
  StampedTransform(const tf::Transform &input, const fawkes::Time &timestamp,
                   const std::string &frame_id, const std::string &child_frame_id)
  : tf::Transform(input), stamp(timestamp),
    frame_id(frame_id), child_frame_id(child_frame_id)
  {};

  
  /** Default constructor only to be used for preallocation */
  StampedTransform() {};

  /** Set the inherited Transform data.
   * @param input transform to set
   */
  void set_data(const tf::Transform &input)
  { *static_cast<tf::Transform*>(this) = input; };
};


/** Wrapper class to add time stamp and frame ID to base types. */
template <typename T>
class Stamped : public T{
 public:
  fawkes::Time stamp; ///< The timestamp associated with this data
  std::string frame_id; ///< The frame_id associated this data

  /** Default constructor.
   * Default constructor used only for preallocation.
   */
  Stamped() :frame_id ("NO_ID_STAMPED_DEFAULT_CONSTRUCTION"){};

  /** Constructor.
   * @param input transform
   * @param timestamp timestamp for this transform
   * @param frame_id frame ID the transform is relative to
   */
  Stamped(const T &input, const fawkes::Time &timestamp,
          const std::string &frame_id)
    : T(input), stamp(timestamp), frame_id(frame_id) {};

  /** Set the data element.
   * @param input data to set this instance to
   */
  void set_data(const T& input){*static_cast<T*>(this) = input;};
};



/** Comparison operator for StampedTransform.
 * @param a transform to compare
 * @param b transform to compare
 * @return true of the transforms are the same, i.e. the parent and
 * child frame IDs between the transforms are the same, as well as the
 * time stamps and transforms.
 */
static inline bool operator==(const StampedTransform &a, const StampedTransform &b)
{
  return
    a.frame_id == b.frame_id &&
    a.child_frame_id == b.child_frame_id &&
    a.stamp == b.stamp &&
    static_cast<const Transform&>(a) == static_cast<const Transform&>(b);
};


/** \brief Throw InvalidArgument if quaternion is malformed */
inline void
assert_quaternion_valid(const Quaternion & q)
{
  if (std::isnan(q.x()) || std::isnan(q.y()) ||
      std::isnan(q.z()) || std::isnan(q.w()))
  {
    throw InvalidArgumentException("Quaternion malformed, contains NaN value");
  }

  double magnitude = q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w();
  if(std::fabs(magnitude - 1) > 0.01) {
    throw InvalidArgumentException("Quaternion malformed, magnitude: %f, "
                                   "should be 1.0", magnitude);
  }
};

/** Construct a Quaternion from fixed angles.
 * @param roll The roll about the X axis
 * @param pitch The pitch about the Y axis
 * @param yaw The yaw about the Z axis
 * @return The quaternion constructed
 */
static inline Quaternion
create_quaternion_from_rpy(double roll, double pitch, double yaw)
{
  Quaternion q;
  q.setEulerZYX(yaw, pitch, roll);
  return q;
}

/** Construct a Quaternion from yaw only.
 * @param yaw The yaw about the Z axis
 * @return The quaternion constructed
 */
static inline Quaternion
create_quaternion_from_yaw(double yaw)
{
  Quaternion q;
  q.setEulerZYX(yaw, 0.0, 0.0);
  return q;
}


/** Helper function for getting yaw from a Quaternion.
 * @param bt_q quaternion to get yaw from
 * @return yaw value
 */
static inline double get_yaw(const Quaternion& bt_q){
  Scalar useless_pitch, useless_roll, yaw;
  Matrix3x3(bt_q).getEulerZYX(yaw, useless_pitch, useless_roll);
  return yaw;
}

/** Helper function for getting yaw from a pose
 * @param t pose to get yaw from
 * @return yaw value
 */
static inline double get_yaw(Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerZYX(yaw,pitch,roll);
  return yaw;
}


} // end namespace tf
} // end namespace fawkes

#endif
