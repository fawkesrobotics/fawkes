
/***************************************************************************
 *  mirrormodel.cpp - Abstract class defining a mirror model
 *
 *  Created: Wed Mar 21 16:32:32 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
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

#include <fvmodels/mirror/mirrormodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MirrorModel <fvmodels/mirror/mirrormodel.h>
 * Mirror model interface.
 * This interface defines the API for a mirror model.
 *
 * @fn void MirrorModel::warp2unwarp(unsigned int warp_x, unsigned int warp_y, unsigned int *unwarp_x, unsigned int *unwarp_y)
 * Transform warped to unwarped point.
 * Given a point in the warped image it returns the coordinates of
 * the corresponding pixel in the unwarped image
 * Useful for: You found the ball center in the image and want to get
 * the position this pixel would have in an unwarped image
 * @param warp_x warped x coordinate
 * @param warp_y warped y coordinate
 * @param unwarp_x contains unwarped x coordinate upon return
 * @param unwarp_y contains unwarped y coordinate upon return
 *
 * @fn void MirrorModel::unwarp2warp(unsigned int unwarp_x, unsigned int unwarp_y, unsigned int *warp_x, unsigned int *warp_y)
 * Transform unwarped to warped point.
 * Given a point in the unwarped image it returns the coordinates of
 * the corresponding pixel in the warped image
 * Useful for: You want to generate the unwarped image and ask the model
 * for every point of the unwarped image which warped pixel to copy
 * @param unwarp_x unwarped x coordinate
 * @param unwarp_y unwarped y coordinate
 * @param warp_x contains the warped x coordinate upon return
 * @param warp_y contains the warped y coordinate upon return
 *
 * @fn const char * MirrorModel::getName()
 * Get name of model.
 * @return name of model
 *
 * @fn polar_coord_t MirrorModel::getWorldPointRelative(unsigned int image_x, unsigned int image_y  ) const
 * Get relative coordinate based on image coordinates.
 * @param image_x x coordinate in image in pixels
 * @param image_y y coordinate in image in pixels
 * @return polar coordinates relative to the base system in metric local system
 *
 * @fn f_point_t MirrorModel::getWorldPointGlobal(unsigned int image_x, unsigned int image_y, float pose_x, float pose_y, float pose_ori) const
 * Get global coordinate based on image coordinates.
 * @param image_x x coordinate in image in pixels
 * @param image_y y coordinate in image in pixels
 * @param pose_x robot pose global x coordinate
 * @param pose_y robot pose global y coordinate
 * @param pose_ori robot pose global orientation
 * @return cartesian coordinates relative to the base system in metric global system
 *
 * @fn void MirrorModel::reset()
 * Reset model. This will reset mirror model.
 *
 * @fn cart_coord_t MirrorModel::getCenter() const
 * Get the image pixel that is the center of the omni-camera.
 * @return pixel coordinates of mirror center in image.
 *
 * @fn  void MirrorModel::setCenter(unsigned int image_x, unsigned int image_y)
 * Set center of omni-camera to given image pixel
 * @param image_x x coordinate in image in pixels
 * @param image_y y coordinate in image in pixels
 *
 * @fn void MirrorModel::setOrientation(float angle)
 * Set orientation of the omni-camera device.
 * @param angle angle to the forward direction.
 *
 * @fn float MirrorModel::getOrientation() const
 * Get orientation of the omni-camera.
 * @return angle to forward direction.
 *
 * @fn bool MirrorModel::isValidPoint(unsigned int image_x, unsigned int image_y ) const
 * Check if the given point is valid.
 * @param image_x x coordinate of queried pixel in image
 * @param image_y y coordinate of queried pixel in image
 * @return true, if pixel is valid, false otherwise.
 */

/** Virtual empty destructor. */
MirrorModel::~MirrorModel()
{
}

} // end namespace firevision
