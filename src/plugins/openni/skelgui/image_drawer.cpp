
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

SkelGuiImageDrawer::SkelGuiImageDrawer(firevision::Camera *cam)
  : __width(cam->pixel_width()), __height(cam->pixel_height())
{
  __cam            = cam;
  __rgb_buf        = malloc_buffer(RGB, __width, __height);

  __texture_width  = get_closest_power_of_two(__width);
  __texture_height = get_closest_power_of_two(__height);

  printf("Dim %ux%u   TexDim %ux%u\n", __width, __height, __texture_width,
	 __texture_height);

  __texture = (unsigned char *)malloc(__texture_width *__texture_height * 3);
  memset(__texture, 0, __texture_width * __texture_height * 3);

  __texture_initialized = false;
}


SkelGuiImageDrawer::~SkelGuiImageDrawer()
{
  free(__texture);
}

unsigned int
SkelGuiImageDrawer::get_closest_power_of_two(unsigned int n)
{
  unsigned int m = 2;
  while(m < n) m<<=1;

  return m;
}

void
SkelGuiImageDrawer::init_texture()
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
SkelGuiImageDrawer::draw_rectangle(float topLeftX, float topLeftY,
				   float bottomRightX, float bottomRightY)
{
  GLfloat verts[8] = {	topLeftX, topLeftY, topLeftX, bottomRightY,
			bottomRightX, bottomRightY, bottomRightX, topLeftY };
  glVertexPointer(2, GL_FLOAT, 0, verts);
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  glFlush();
}

void
SkelGuiImageDrawer::draw_texture()
{
  if (!__texture_initialized) init_texture();

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


void
SkelGuiImageDrawer::draw_camimg()
{
  __cam->capture();

  convert(__cam->colorspace(), RGB, __cam->buffer(), __rgb_buf, __width, __height);

  unsigned char *row = __texture;
  unsigned char *tex = __texture;
  unsigned char *rgb = __rgb_buf;
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

  /*
  unsigned char *tex = __texture;
  static unsigned char r_val = 255;
  static unsigned char g_val = 0;

  r_val = 255 - r_val;
  g_val = 255 - g_val;

  unsigned char r, g;
  for (unsigned int h = 0; h < __texture_height; ++h) {
    //tex = row;
    r = (h % 2 == 1) ? 255 : 0;
    g = (h % 2 == 0) ? 255 : 0;
    for (unsigned int w = 0; w < __texture_width; ++w) {
      tex[0] = r;
      tex[1] = 0;
      tex[2] = 0;
      tex += 3;
    }
  }
  */

  draw_texture();

  __cam->dispose_buffer();
}

