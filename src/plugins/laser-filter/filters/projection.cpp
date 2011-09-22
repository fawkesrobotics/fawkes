
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
#include <utils/math/angle.h>
#include <geometry/hom_polar.h>
#include <geometry/hom_vector.h>

#include <cstdlib>
#include <cstring>
#include <sys/types.h>

using namespace fawkes;

/** @class LaserProjectionDataFilter "filters/projection.h"
 * Projects URG lasers into the EDL laser plane.
 * @author Christoph Schwering
 */

/** @class LaserProjectionDataFilter::Rotation "filters/projection.h"
 * Coordinate system rotation structure.
 * @author Christoph Schwering
 */

/** @fn LaserProjectionDataFilter::Rotation::Rotation(float x_rot_degree, float y_rot_degree, float z_rot_degree) "filters/projection.h"
 * Constructor for a new rotation container.
 * @param x_rot_degree coordinate system rotation around the X axis which is
 *                     performed last (counter-clockwise rotation in degrees)
 * @param y_rot_degree coordinate system rotation around the Y axis which is
 *                     performed last (counter-clockwise rotation in degrees)
 * @param z_rot_degree coordinate system rotation around the Z axis which is
 *                     performed last (counter-clockwise rotation in degrees)
 */

/** @var float LaserProjectionDataFilter::Rotation::x
 * Rotation around X axis in degrees. */

/** @var float LaserProjectionDataFilter::Rotation::y
 * Rotation around X axis in degrees. */

/** @var float LaserProjectionDataFilter::Rotation::z
 * Rotation around X axis in degrees. */

/** @class LaserProjectionDataFilter::Translation "filters/projection.h"
 * Coordinate system translation structure.
 * @author Christoph Schwering
 */

/** @fn LaserProjectionDataFilter::Translation::Translation(float x_trans_degree, float y_trans_degree, float z_trans_degree) "filters/projection.h"
 * Constructor for a new translation container.
 * @param x_trans_degree x-component of the vector from EDL laser to URG laser
 * @param y_trans_degree y-component of the vector from EDL laser to URG laser
 * @param z_trans_degree z-component of the vector from EDL laser to URG laser
 */

/** @var float LaserProjectionDataFilter::Translation::x
 * X componenent of vector from destination to origin. */

/** @var float LaserProjectionDataFilter::Translation::y
 * Y componenent of vector from destination to origin. */

/** @var float LaserProjectionDataFilter::Translation::z
 * Z componenent of vector from destination to origin. */

/** @class LaserProjectionDataFilter::Rectangle "filters/projection.h"
 * Rectangle structure.
 * @author Christoph Schwering
 */

/** @fn LaserProjectionDataFilter::Rectangle::Rectangle(float x_min, float x_max, float y_min, float y_max) "filters/projection.h"
 * Constructor for a new translation container.
 * @param x_min distance from EDL laser to back of robot
 * @param x_max distance from EDL laser to front of robot
 * @param y_min distance from EDL laser to right of robot
 * @param y_max distance from EDL laser to left of robot
 */

/** @var float LaserProjectionDataFilter::Rectangle::x_min
 * Distance from EDL to back of robot. */

/** @var float LaserProjectionDataFilter::Rectangle::x_max
 * Distance from EDL to front of robot. */

/** @var float LaserProjectionDataFilter::Rectangle::y_min
 * Distance from EDL to right of robot. */

/** @var float LaserProjectionDataFilter::Rectangle::y_max
 * Distance from EDL to left of robot. */

/** Constructor.
 * @param in_data_size number of entries input value arrays.
 * @param in vector of input arrays.
 * @param laser_rot the rotation of the X, Y and Z axis of the laser beams
 *                  coordinate system with respect to the laser panel coordinate
 *                  system.
 * @param fixture_rot the rotation of the X, Y and Z axis of the laser panel
 *                    coordinate system with respect to the fawkes coordinate
 *                    system.
 * @param trans the translation from the URG laser into the EDL laser plane,
 *              which is the vector from the EDL laser to the URG laser.
 * @param robot_rectangle a rectangle relative to the fawkes coordinate system
 *                        centered at the EDL laser which denotes the size of
 *                        the robot.
 * @param z_threshold all points with this value as Z coordinate in the EDL
 *                    laser coordinate system are considered ground points
 *                    and therefore ignored; a suitable value might be -0.05
 *                    meters (note that the threshold should be negative,
 *                    because the ground is below the EDL laser).
 */
LaserProjectionDataFilter::LaserProjectionDataFilter(
    const Rotation& laser_rot,
    const Rotation& fixture_rot,
    const Translation& trans,
    const Rectangle& robot_rectangle,
    float z_threshold,
    unsigned int in_data_size,
    std::vector<float *> in)
  : LaserDataFilter(in_data_size, in, in.size()),
    LASER_ROT(laser_rot),
    FIXTURE_ROT(fixture_rot),
    TRANS(trans),
    ROBOT(robot_rectangle),
    Z_THRESHOLD(z_threshold)
{
}

LaserProjectionDataFilter::~LaserProjectionDataFilter()
{
}

inline void
LaserProjectionDataFilter::transform(const float angle, const float length,
                                     float& new_angle, float& new_length,
                                     bool& in_robot_rect, bool& too_low)
{
  HomPolar p = HomPolar(length, angle);
  // 1. Move the coordinate so that subsequent rotations are exactly like the
  // fixtures.
  p.rotate_z(-1.0f * deg2rad(LASER_ROT.z));
  p.rotate_y(-1.0f * deg2rad(LASER_ROT.y));
  p.rotate_x(-1.0f * deg2rad(LASER_ROT.x));
  // 2. Rotate the coordinate system the same way it was rotated by the
  // fixtures.
  p.rotate_z(-1.0f * deg2rad(FIXTURE_ROT.z));
  p.rotate_y(-1.0f * deg2rad(FIXTURE_ROT.y));
  p.rotate_x(-1.0f * deg2rad(FIXTURE_ROT.x));
  // 3. Translate to the position of the EDL laser.
  p += HomVector(TRANS.x, TRANS.y, TRANS.z);
  in_robot_rect = ROBOT.x_min < p.x() && p.x() < ROBOT.x_max &&
                  ROBOT.y_min < p.y() && p.y() < ROBOT.y_max;
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
      bool in_robot_rect;
      bool too_low;
      transform(angle, length, new_angle, new_length, in_robot_rect, too_low);
      if (in_robot_rect) {
        // skip readings that are within the X, Y coords of the robot
        continue;
      }
      if (too_low) {
        // skip readings that probably hit the ground
        continue;
      }
      const int j = static_cast<int>(rad2deg(normalize_rad(new_angle)));
      outbuf[j] = new_length;
    }
  }
}

