
/***************************************************************************
 *  conversions.h - OpenNI utility methods: conversions
 *
 *  Created: Thu Mar 31 21:22:19 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <plugins/openni/utils/conversions.h>

#include <cmath>

namespace fawkes {
  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** Project world coordinate into 2D image projection.
 * This takes the input world coordinates and projects them into the 2D
 * image plane. Unlike the OpenNI DepthGenerator function this function
 * can accept a custom width and height. This erases the limitation to
 * be bound to the configured depth image dimensions, and rather use
 * coordinates as anticipated from, say, a GUI.
 * @param depthgen depth generator, used to get field of view and possibly
 * width and height
 * @param num_points number of points to convert
 * @param world array of @p num_points world points
 * @param proj array of @p num_points projective points
 * @param width width of the image, 0 to use the actual value from the depth
 * generator
 * @param height height of the image, 0 to use the actual value from the depth
 * generator
 */
void
world2projection(xn::DepthGenerator *depthgen,
		 unsigned int num_points, const XnPoint3D *world, XnPoint3D *proj,
		 unsigned int width, unsigned int height)
{
  if (width == 0 || height == 0) {
    xn::DepthMetaData depth_md;
    depthgen->GetMetaData(depth_md);
    width  = depth_md.XRes();
    height = depth_md.YRes();
  }
  
  XnFieldOfView fov;
  XnStatus st;
  if ((st = depthgen->GetFieldOfView(fov)) != XN_STATUS_OK) {
    throw Exception("Failed to get field of view, ignoring. (%s)",
		    xnGetStatusString(st));
  }

  float world_x_to_z = tan(fov.fHFOV / 2) * 2;;
  float world_y_to_z = tan(fov.fVFOV / 2) * 2;

  XnFloat coeff_x = width  / world_x_to_z;
  XnFloat coeff_y = height / world_y_to_z;

  XnUInt32 half_res_x = width  / 2;
  XnUInt32 half_res_y = height / 2;

  for (unsigned int i = 0; i < num_points; ++i) {
    proj[i].X = coeff_x * world[i].X / world[i].Z + half_res_x;
    proj[i].Y = half_res_y - coeff_y * world[i].Y / world[i].Z;
    proj[i].Z = world[i].Z;
  }
}



} // end namespace fawkes::openni
} // end namespace fawkes
