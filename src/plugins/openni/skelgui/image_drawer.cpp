
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
  __cam            = cam;
  __rgb_buf        = malloc_buffer(RGB, __width, __height);

}

/** Destructor. */
SkelGuiImageDrawer::~SkelGuiImageDrawer()
{
  free(__rgb_buf);
}

/** Fill texture. */
void
SkelGuiImageDrawer::fill_texture()
{
  __cam->capture();
  convert(__cam->colorspace(), RGB, __cam->buffer(), __rgb_buf, __width, __height);
  copy_rgb_to_texture(__rgb_buf);
  __cam->dispose_buffer();
}

