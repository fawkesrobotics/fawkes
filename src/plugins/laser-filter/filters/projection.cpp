
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
 * Erase dead spots (i.e. mounting rods in the laser range) from laser data.
 * This filter reads a number of values stored in /hardware/laser/projection, where
 * each dead spot must contain two entries, a start and an end in degrees. Each
 * entry is stored as submembers of the given tree, for example as
 * /hardware/laser/deadspots/0/start and /hardware/laser/deadspots/0/end.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config configuration instance
 * @param logger logger for informational output
 * @param prefix configuration prefix where to log for config information
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 */
LaserProjectionDataFilter::LaserProjectionDataFilter(
    fawkes::Configuration *config,
    fawkes::Logger *logger,
    unsigned int in_data_size,
    std::vector<float *> in)
  : LaserDataFilter(in_data_size, in, in.size()),
    __logger(logger),
    LEFT(false),
    X_ROT(deg2rad(LEFT ? 90 : -90)),
    Y_ROT(deg2rad(LEFT ? -51 : -51)),
    Z_ROT(deg2rad(LEFT ? -38 : 38)),
    X_TRANS(6.0f),
    Y_TRANS(LEFT ? 15.0f : -15.0f),
    Z_TRANS(156.5 - 29.0f)
{
}

LaserProjectionDataFilter::~LaserProjectionDataFilter()
{
}

void
LaserProjectionDataFilter::transform(const float angle, const float length,
                                     float& new_angle, float& new_length)
{
  HomPolar p = HomPolar(length, angle);
  p.rotate_z(-1.0f * Z_ROT).rotate_y(-1.0f * Y_ROT).rotate_x(-1.0f * X_ROT);
  p += HomVector(X_TRANS, Y_TRANS, Z_TRANS);
  p.z() = 0.0f;
  new_angle = p.phi();
  new_length = p.length();
  // Why is p.length() != p.r() (p.r() == 160)
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
      if (new_length != 0.0f) {
        printf("converted %d/%.2f into %d/%.2f\n", i, length, j, new_length);
      }
      outbuf[j] = new_length;
    }
  }
}

