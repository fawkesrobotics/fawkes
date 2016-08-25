
/***************************************************************************
 *  projection.cpp - Laser data projection filter
 *
 *  Created: Tue Mar 22 16:30:51 2011
 *  Copyright  2011  Christoph Schwering
 *             2011  Tim Niemueller
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

#include <cstdlib>
#include <cstring>
#include <sys/types.h>

using namespace fawkes;

/** @class LaserProjectionDataFilter "filters/projection.h"
 * Projects one laser into another laser's plane.
 * This first transforms the laser value into the target frame, and then
 * projects it into the X-Y plane (which is assumed to be the laser plane
 * of another (virtual) laser. Additionally some sanity filtering in all
 * three axes is applied after the transformation, but before the
 * projection.
 * @author Tim Niemueller, Christoph Schwering
 */

/** Constructor.
 * @param filter_name name of this filter
 * @param tf transformer to get transform from
 * @param target_frame target coordinate fram to project into
 * @param not_from_x lower X boundary of ignored rectangle
 * @param not_to_x upper X boundary of ignored rectangle
 * @param not_from_y lower Y boundary of ignored rectangle
 * @param not_to_y upper Y boundary of ignored rectangle
 * @param only_from_z minimum Z value for accepted points
 * @param only_to_z maximum Z value for accepted points
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 */
LaserProjectionDataFilter::LaserProjectionDataFilter(
    const std::string filter_name,
    tf::Transformer *tf,
    std::string target_frame,
    float not_from_x, float not_to_x,
    float not_from_y, float not_to_y,
    float only_from_z, float only_to_z,
    unsigned int in_data_size,
    std::vector<LaserDataFilter::Buffer *> &in)
	: LaserDataFilter(filter_name, in_data_size, in, in.size()),
    tf_(tf), target_frame_(target_frame),
    not_from_x_(not_from_x), not_to_x_(not_to_x),
    not_from_y_(not_from_y), not_to_y_(not_to_y),
    only_from_z_(only_from_z), only_to_z_(only_to_z)
{
  // Generate lookup tables for sin and cos
  for (unsigned int i = 0; i < 360; ++i) {
    sin_angles360[i] = sinf(deg2rad(i));
    cos_angles360[i] = cosf(deg2rad(i));
  }
  for (unsigned int i = 0; i < 720; ++i) {
    sin_angles720[i] = sinf(deg2rad((float)i / 2.));
    cos_angles720[i] = cosf(deg2rad((float)i / 2.));
  }

  index_factor_ = out_data_size / 360.;
}

LaserProjectionDataFilter::~LaserProjectionDataFilter()
{
}


/** Set the output buffer applying filtering.
 * This checks the given point against the configured bounds. If and
 * only if the point satisfies the given criteria it is set at the
 * appropriate array index of the buffer.
 * @param outbuf buffer in which to set the value conditionally
 * @param p point to check and (maybe) set
 */
inline void
LaserProjectionDataFilter::set_output(float *outbuf, fawkes::tf::Point &p)
{
  if ( ((p.x() >= not_from_x_) && (p.x() <= not_to_x_) &&
        (p.y() >= not_from_y_) && (p.y() <= not_to_y_)) ||
       (p.z() < only_from_z_) || (p.z() > only_to_z_) )
  {
    // value is inside "forbidden" robot rectangle or
    // below or above Z thresholds
    return;
  }

  p.setZ(0.);
  float phi    = atan2f(p.y(), p.x());
    
  unsigned int j =
    (unsigned int)roundf(rad2deg(normalize_rad(phi)) * index_factor_);
  if (j > out_data_size) j = 0; // might happen just at the boundary

  if (outbuf[j] == 0.) {
    outbuf[j] = (float)p.length();
  } else {
    outbuf[j] = std::min(outbuf[j], (float)p.length());
  }
}

void
LaserProjectionDataFilter::filter()
{
  const unsigned int vecsize = std::min(in.size(), out.size());
  for (unsigned int a = 0; a < vecsize; ++a) {
    out[a]->frame = target_frame_;
    out[a]->timestamp->set_time(in[a]->timestamp);
    float* inbuf  = in[a]->values;
    float* outbuf = out[a]->values;
    memset(outbuf, 0, sizeof(float) * out_data_size);

    tf::StampedTransform t;
    tf::Point p;

    tf_->lookup_transform(target_frame_, in[a]->frame,
			  fawkes::Time(0, 0), t);

    if (in_data_size == 360) {
      for (unsigned int i = 0; i < 360; ++i) {
        if (inbuf[i] == 0.)  continue;
        p.setValue(inbuf[i] * cos_angles360[i], inbuf[i] * sin_angles360[i], 0.);
        p = t * p;

        set_output(outbuf, p);
      }
    } else if (in_data_size == 720) {
      for (unsigned int i = 0; i < 720; ++i) {
        if (inbuf[i] == 0.)  continue;

        p.setValue(inbuf[i] * cos_angles720[i], inbuf[i] * sin_angles720[i], 0.);
        p = t * p;

        set_output(outbuf, p);
      }
    } else {
      for (unsigned int i = 0; i < in_data_size; ++i) {
        if (inbuf[i] == 0.)  continue;

        float a = deg2rad(360.f / (float)i);
        p.setValue(inbuf[i] * cos(a), inbuf[i] * sin(a), 0.);
        p = t * p;

        set_output(outbuf, p);
      }
    }
  }
}

