
/***************************************************************************
 *  image_drawer.cpp - Skeleton Visualization GUI: image drawer
 *
 *  Created: Sat Mar 19 00:08:37 2011
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

#include "image_drawer.h"

#include <fvcams/camera.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>

#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <GL/glut.h>

using namespace fawkes;
using namespace firevision;

/** @class SkelGuiImageDrawer "image_drawer.h"
 * Draw images from camera in texture.
 * Uses texture mapping to show an image acquired from a camera in the
 * background.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cam camera to capture image with
 */
SkelGuiImageDrawer::SkelGuiImageDrawer(firevision::Camera *cam)
  : SkelGuiTextureDrawer(cam->pixel_width(), cam->pixel_height())
{
  cam_            = cam;
  rgb_buf_        = malloc_buffer(RGB, width_, height_);

}

/** Destructor. */
SkelGuiImageDrawer::~SkelGuiImageDrawer()
{
  free(rgb_buf_);
}

/** Fill texture. */
void
SkelGuiImageDrawer::fill_texture()
{
  cam_->capture();
  convert(cam_->colorspace(), RGB, cam_->buffer(), rgb_buf_, width_, height_);
  copy_rgb_to_texture(rgb_buf_);
  cam_->dispose_buffer();
}

