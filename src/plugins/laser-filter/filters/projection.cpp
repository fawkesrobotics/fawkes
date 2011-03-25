
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
 * @param config configuration instance
 * @param logger logger for informational output
 * @param prefix configuration prefix where to log for config information
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
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
    Z_TRANS(z_trans)
{
}

LaserProjectionDataFilter::~LaserProjectionDataFilter()
{
}

inline void
LaserProjectionDataFilter::transform(const float angle, const float length,
                                     float& new_angle, float& new_length)
{
  HomPolar p = HomPolar(length, angle);
  // 1. Move the coordinate so that subsequent rotations are exactly like the
  // fixtures.
  if (LEFT) {
    p.rotate_z(-1.0f * deg2rad(90.0f)).rotate_y(-1.0f * deg2rad(-90.0f));
  } else {
    p.rotate_z(-1.0f * deg2rad(-90.0f)).rotate_y(-1.0f * deg2rad(-90.0f));
  }
  // 2. Rotate the coordinate system the same way it was rotated by the
  // fixtures.
  p.rotate_z(-1.0f * Z_ROT).rotate_y(-1.0f * Y_ROT).rotate_x(-1.0f * X_ROT);
  // 3. Translate to the position of the EDL laser.
  p += HomVector(X_TRANS, Y_TRANS, Z_TRANS);
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
      transform(angle, length, new_angle, new_length);
      const int j = static_cast<int>(rad2deg(normalize_rad(new_angle)));
      outbuf[j] = new_length;
    }
  }
}

