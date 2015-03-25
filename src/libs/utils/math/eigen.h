
/***************************************************************************
 *  eigen_utils.h - Utils related to Eigen3
 *
 *  Created: Wed Mar 25 14:40:14 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_MATH_EIGEN3_H_
#define __UTILS_MATH_EIGEN3_H_

#include <Eigen/Geometry>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** Calculate Yaw angle from quaternion.
 * The Yaw angle is the rotation around the Z axis of a given reference
 * frame.
 * Code based on OpenSLAM.
 * @param q quaternion to get yaw angle for
 * @return yaw angle
 */
template <typename Scalar>
Scalar
quat_yaw(const Eigen::Quaternion<Scalar> &q)
{
  Scalar qx=q.x(), qy=q.y(), qz=q.z(), qw=q.w();
  Scalar qx2=qx*qx, qy2=qy*qy, qz2=qz*qz, qw2=qw*qw;
  // for abs(pitch) = PI/2 this will lead to atan2(0,0)
  // i.e. for noisy values, result will be arbitrary
  return atan2(2*(qw*qz + qx*qy), qw2 + qx2 - qy2 - qz2);
}

/** Get euler angles for quaternion.
 * Calculates the roll, pitch, and yaw angles for a given quaternion.
 * Code based on OpenSLAM.
 * @param q quaternion to convert
 * @param roll upon return contains roll angle (around X axis)
 * @param pitch upon return contains pitch angle (around Y axis)
 * @param yaw upon return contains yaw angle (around Z axis)
 */
template <typename Scalar>
void
quat_to_euler(const Eigen::Quaternion<Scalar> &q, float &roll, float &pitch, float &yaw)
{
  using std::cos;
  using std::sin;
  using std::atan2;
	
  // Get yaw angle:
  Scalar qx=q.x(), qy=q.y(), qz=q.z(), qw=q.w();
  Scalar qx2=qx*qx, qy2=qy*qy, qz2=qz*qz, qw2=qw*qw;
  // for abs(pitch) = PI/2 this will lead to atan2(0,0)
  // i.e. for noisy values, result will be arbitrary
  yaw = atan2(2*(qw*qz + qx*qy), qw2 + qx2 - qy2 - qz2);
	
  // Now rotate the original Quaternion backwards by yaw:
  Scalar c = cos(yaw/2), s=sin(yaw/2);
  Scalar px=c*qx+s*qy, py=c*qy-s*qx, pz=c*qz-s*qw, pw=c*qw+s*qz;
  Scalar px2=px*px, py2=py*py, pz2=pz*pz, pw2=pw*pw;
	
  // Now calculating pitch and roll does not have singularities anymore:
  pitch = atan2(2*(py*pw - px*pz), px2 + pw2 - py2 - pz2);
  roll  = atan2(2*(px*pw - py*pz), py2 + pw2 - px2 - pz2);
}


} // end namespace fawkes

#endif
