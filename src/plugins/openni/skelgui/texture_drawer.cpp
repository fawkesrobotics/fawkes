
/***************************************************************************
 *  texture_drawer.cpp - Skeleton Visualization GUI: texture drawer
 *
 *  Created: Tue Mar 29 17:09:25 2011 (on the way to Magdeburg for GO2011)
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

#include "texture_drawer.h"

#include <fvcams/camera.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>

#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <GL/glut.h>

using namespace fawkes;
using namespace firevision;

/** @class SkelGuiTextureDrawer "texture_drawer.h"
 * Draw images from camera in texture.
 * Uses texture mapping to show an image acquired from a camera in the
 * background.
 * @author Tim Niemueller
 *
 * @fn SkelGuiTextureDrawer::fill_texture()
 * Fill texture with data.
 * This function is called during draw() and the sub-class shall implement it
 * to fill the texture with the data to show. Be aware that the texture size
 * and the actually shown size will likely differ.
 */

/** Constructor.
 * @param width width of visible area
 * @param height height of visible area
 */
SkelGuiTextureDrawer::SkelGuiTextureDrawer(unsigned int width, unsigned int height)
  : __width(width), __height(height),
    __texture_width(get_closest_power_of_two(__width)),
    __texture_height(get_closest_power_of_two(__height))
{
  __texture = (unsigned char *)malloc(__texture_width *__texture_height * 3);
  memset(__texture, 0, __texture_width * __texture_height * 3);

  __texture_initialized = false;
}

/** Destructor. */
SkelGuiTextureDrawer::~SkelGuiTextureDrawer()
{
  free(__texture);
}

unsigned int
SkelGuiTextureDrawer::get_closest_power_of_two(unsigned int n)
{
  unsigned int m = 2;
  while(m < n) m<<=1;

  return m;
}

void
SkelGuiTextureDrawer::init_texture()
{
  glGenTextures(1, &__texture_id);
  glBindTexture(GL_TEXTURE_2D, __texture_id);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  memset(__texture_coords, 0, sizeof(__texture_coords));
  __texture_coords[0] = (float)__width  / __texture_width;
  __texture_coords[1] = (float)__height / __texture_height;
  __texture_coords[2] = (float)__width  / __texture_width;
  __texture_coords[7] = (float)__height / __texture_height;

  __texture_initialized = true;
}


void
SkelGuiTextureDrawer::draw_rectangle(float topLeftX, float topLeftY,
				   float bottomRightX, float bottomRightY)
{
  GLfloat verts[8] = {	topLeftX, topLeftY, topLeftX, bottomRightY,
			bottomRightX, bottomRightY, bottomRightX, topLeftY };
  glVertexPointer(2, GL_FLOAT, 0, verts);
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  glFlush();
}

/** Draw texture to screen. */
void
SkelGuiTextureDrawer::draw()
{
  if (!__texture_initialized) init_texture();

  fill_texture();

  glBindTexture(GL_TEXTURE_2D, __texture_id);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, __texture_width, __texture_height,
	       0, GL_RGB, GL_UNSIGNED_BYTE, __texture);

  // Display the OpenGL texture map
  glColor4f(0.75,0.75,0.75,1);

  glEnable(GL_TEXTURE_2D);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  glTexCoordPointer(2, GL_FLOAT, 0, __texture_coords);
  draw_rectangle(__width, __height, 0, 0);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisable(GL_TEXTURE_2D);
}

/** Copy an RGB buffer to texture.
 * @param rgb_buf the RGB buffer to copy, it must exactly of dimensions __width
 * and __height.
 */
void
SkelGuiTextureDrawer::copy_rgb_to_texture(const unsigned char *rgb_buf)
{
  unsigned char *row = __texture;
  unsigned char *tex = __texture;
  const unsigned char *rgb = rgb_buf;
  unsigned int bytes = 0;
  for (unsigned int h = 0; h < __height; ++h) {
    tex = row;
    for (unsigned int w = 0; w < __width; ++w) {
      *tex++ = *rgb++;
      *tex++ = *rgb++;
      *tex++ = *rgb++;
      ++bytes;
    }
    row += __texture_width * 3;
  }
}
