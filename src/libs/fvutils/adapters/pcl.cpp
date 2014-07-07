
/***************************************************************************
 *  pcl.cpp - Convert PCL buffer to PointCloud structure
 *
 *  Created: Wed Nov 02 21:17:35 2011
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

#include <fvutils/adapters/pcl.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>
#include <core/exception.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

void
convert_buffer_to_pcl(const SharedMemoryImageBuffer *buffer,
                      pcl::PointCloud<pcl::PointXYZ> &pcl)
{
  if (buffer->colorspace() != CARTESIAN_3D_FLOAT) {
    throw Exception("Invalid colorspace, expected CARTESIAN_3D_FLOAT");
  }

  const pcl_point_t *pclbuf = (const pcl_point_t*)buffer->buffer();

  const unsigned int width  = buffer->width();
  const unsigned int height = buffer->height();

  pcl.height = height;
  pcl.width = width;
  pcl.is_dense = false;
  pcl.points.resize(width * height);

  for (unsigned int i = 0; i < width * height; ++i) {
    pcl::PointXYZ &p = pcl.points[i];
    const pcl_point_t &pt = pclbuf[i];
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
  }

}


void
convert_buffer_to_pcl(const SharedMemoryImageBuffer *buffer,
                      pcl::PointCloud<pcl::PointXYZRGB> &pcl)
{
  if (buffer->colorspace() != CARTESIAN_3D_FLOAT) {
    throw Exception("Invalid colorspace, expected CARTESIAN_3D_FLOAT");
  }

  const pcl_point_t *pclbuf = (const pcl_point_t*)buffer->buffer();

  const unsigned int width  = buffer->width();
  const unsigned int height = buffer->height();

  pcl.height = height;
  pcl.width = width;
  pcl.is_dense = false;
  pcl.points.resize(width * height);

  for (unsigned int i = 0; i < width * height; ++i) {
    pcl::PointXYZRGB &p = pcl.points[i];
    const pcl_point_t &pt = pclbuf[i];
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.r = p.g = p.b = 255;
  }
}


} // end namespace firevision
