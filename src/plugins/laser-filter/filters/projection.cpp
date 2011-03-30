
/***************************************************************************
 *  projection.cpp - Laser data projection filter
 *
 *  Created: Tue Mar 22 16:30:51 2011
 *  Copyright  2011  Christoph Schwering
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "projection.h"

#include <core/exception.h>
#include <core/macros.h>
#include <utils/math/angle.h>
#include <utils/logging/logger.h>
#include <config/config.h>
#include <geometry/hom_polar.h>
#include <geometry/hom_vector.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <regex.h>

using namespace fawkes;

/** @class LaserProjectionDataFilter "filters/projection.h"
 * Projects URG lasers into the EDL laser plane.
 * @author Christoph Schwering
 */

/** Constructor.
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 * @param left indicates whether or not the laser at the left or at the right
 *             of the robot or, in other words: left must be true iff the X axis
 *             of the laser is directed to the bottom right.
 *             <br/>
 *             This information is needed to know how to rotate the laser's
 *             coordinate system such that the X axis comes out of the front
 *             panel and the Y axis comes out of the right side panel of the
 *             laser body.
 *             <br/>
 *             Subsequently, this coordinate system is rotated (see [xyz]_rot)
 *             and then translated ([xyz]_trans).
 * @param x_rot the rotation of the X axis of the laser panel coordinate system
 *              which is determined with the left parameter (see left).
 *              This rotation is done after the Y axis rotation.
 * @param y_rot the rotation of the Y axis of the laser panel coordinate system
 *              which is determined with the left parameter (see left).
 *              This rotation is done before the Z axis rotation and
 *              after the Y axis rotation.
 * @param z_rot the rotation of the Z axis of the laser panel coordinate system
 *              which is determined with the left parameter (see left).
 *              This rotation is done before the Y axis and X axis rotations.
 * @param x_trans X component of the vector from the EDL laser to this URG
 *                laser in the fawkes coordinate system.
 * @param y_trans Y component of the vector from the EDL laser to this URG
 *                laser in the fawkes coordinate system.
 * @param z_trans Z component of the vector from the EDL laser to this URG
 *                laser in the fawkes coordinate system.
 */
LaserProjectionDataFilter::LaserProjectionDataFilter(
    bool left,
    float x_rot, float y_rot, float z_rot,
    float x_trans, float y_trans, float z_trans,
    unsigned int in_data_size,
    std::vector<float *> in)
  : LaserDataFilter(in_data_size, in, in.size()),
    LEFT(left),
    X_ROT(x_rot),
    Y_ROT(y_rot),
    Z_ROT(z_rot),
    X_TRANS(x_trans),
    Y_TRANS(y_trans),
    Z_TRANS(z_trans),
    Z_THRESHOLD(-0.15f)
{
}

LaserProjectionDataFilter::~LaserProjectionDataFilter()
{
}

inline void
LaserProjectionDataFilter::transform(const float angle, const float length,
                                     float& new_angle, float& new_length,
                                     bool& too_low)
{
  HomPolar p = HomPolar(length, angle);
  // 1. Move the coordinate so that subsequent rotations are exactly like the
  // fixtures.
  if (LEFT) {
    p.rotate_z(-1.0f * deg2rad(90.0f));
    p.rotate_y(-1.0f * deg2rad(-90.0f));
  } else {
    p.rotate_z(-1.0f * deg2rad(-90.0f));
    p.rotate_y(-1.0f * deg2rad(-90.0f));
  }
  // 2. Rotate the coordinate system the same way it was rotated by the
  // fixtures.
  p.rotate_z(-1.0f * deg2rad(Z_ROT));
  p.rotate_y(-1.0f * deg2rad(Y_ROT));
  p.rotate_x(-1.0f * deg2rad(X_ROT));
  // 3. Translate to the position of the EDL laser.
  p += HomVector(X_TRANS, Y_TRANS, Z_TRANS);
  too_low = p.z() < Z_THRESHOLD;
  // 4. Cut z-coordinate.
  p.z() = 0.0f;
  new_angle = p.phi();
  new_length = p.length();
  // Why is p.length() != p.r() (p.r() == 160) TODO trac entry
}

void
LaserProjectionDataFilter::filter()
{
  const unsigned int vecsize = std::min(in.size(), out.size());
  for (unsigned int a = 0; a < vecsize; ++a) {
    float* inbuf  = in[a];
    float* outbuf = out[a];
    memset(outbuf, 0, sizeof(float) * out_data_size);
    for (unsigned int i = 0; i < in_data_size; ++i) {
      const float length = inbuf[i];
      if (length == 0.0f) {
        // skip non-readings (they should not be translated, because then the
        // length would not be 0.0f anymore)
        continue;
      }
      const float angle = deg2rad(static_cast<float>(i));
      float new_angle;
      float new_length;
      bool too_low;
      transform(angle, length, new_angle, new_length, too_low);
      if (too_low) {
        // skip readings that probably hit the ground
        continue;
      }
      const int j = static_cast<int>(rad2deg(normalize_rad(new_angle)));
      outbuf[j] = new_length;
    }
  }
}

