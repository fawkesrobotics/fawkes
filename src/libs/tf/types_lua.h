
/***************************************************************************
 *  types_lua.h - Fawkes Lua compatibility tf types
 *
 *  Created: Mon Nov 28 13:22:29 2011
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

#ifndef __LIBS_TF_TYPES_LUA_H_
#define __LIBS_TF_TYPES_LUA_H_

#include <tf/types.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

// The following classes are used for better Lua compatibility

/// @cond LUA_COMPAT

class StampedQuaternion : public Stamped<Quaternion>
{
 public:
 StampedQuaternion() : Stamped<Quaternion>() {}
  StampedQuaternion(const Quaternion &input, const fawkes::Time &timestamp,
                    const std::string &frame_id)
    : Stamped<Quaternion>(input, timestamp, frame_id) {}
};

class StampedVector3 : public Stamped<Vector3>
{
 public:
 StampedVector3() : Stamped<Vector3>() {}
  StampedVector3(const Vector3 &input, const fawkes::Time &timestamp,
                    const std::string &frame_id)
    : Stamped<Vector3>(input, timestamp, frame_id) {}

};


class StampedPoint : public Stamped<Point>
{
 public:
 StampedPoint() : Stamped<Point>() {}
  StampedPoint(const Point &input, const fawkes::Time &timestamp,
                    const std::string &frame_id)
    : Stamped<Point>(input, timestamp, frame_id) {}

};

class StampedPose : public Stamped<Pose>
{
 public:
 StampedPose() : Stamped<Pose>() {}
  StampedPose(const Pose &input, const fawkes::Time &timestamp,
                    const std::string &frame_id)
    : Stamped<Pose>(input, timestamp, frame_id) {}
};

/// @endcond

} // end namespace tf
} // end namespace fawkes

#endif
