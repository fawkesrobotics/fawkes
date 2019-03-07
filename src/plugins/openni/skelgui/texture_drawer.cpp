
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

#include <GL/glut.h>
#include <fvcams/camera.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>

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
: width_(width),
  height_(height),
  texture_width_(get_closest_power_of_two(width_)),
  texture_height_(get_closest_power_of_two(height_)),
  texture_raii_(malloc(texture_width_ * texture_height_ * 3))
{
	texture_ = (unsigned char *)*texture_raii_;
	memset(texture_, 0, texture_width_ * texture_height_ * 3);

	texture_initialized_ = false;
}

/** Destructor. */
SkelGuiTextureDrawer::~SkelGuiTextureDrawer()
{
}

unsigned int
SkelGuiTextureDrawer::get_closest_power_of_two(unsigned int n)
{
	unsigned int m = 2;
	while (m < n)
		m <<= 1;

	return m;
}

void
SkelGuiTextureDrawer::init_texture()
{
	glGenTextures(1, &texture_id_);
	glBindTexture(GL_TEXTURE_2D, texture_id_);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	memset(texture_coords_, 0, sizeof(texture_coords_));
	texture_coords_[0] = (float)width_ / texture_width_;
	texture_coords_[1] = (float)height_ / texture_height_;
	texture_coords_[2] = (float)width_ / texture_width_;
	texture_coords_[7] = (float)height_ / texture_height_;

	texture_initialized_ = true;
}

void
SkelGuiTextureDrawer::draw_rectangle(float topLeftX,
                                     float topLeftY,
                                     float bottomRightX,
                                     float bottomRightY)
{
	GLfloat verts[8] = {
	  topLeftX, topLeftY, topLeftX, bottomRightY, bottomRightX, bottomRightY, bottomRightX, topLeftY};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	glFlush();
}

/** Draw texture to screen. */
void
SkelGuiTextureDrawer::draw()
{
	if (!texture_initialized_)
		init_texture();

	fill_texture();

	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glTexImage2D(GL_TEXTURE_2D,
	             0,
	             GL_RGB,
	             texture_width_,
	             texture_height_,
	             0,
	             GL_RGB,
	             GL_UNSIGNED_BYTE,
	             texture_);

	// Display the OpenGL texture map
	glColor4f(0.75, 0.75, 0.75, 1);

	glEnable(GL_TEXTURE_2D);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, texture_coords_);
	draw_rectangle(width_, height_, 0, 0);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisable(GL_TEXTURE_2D);
}

/** Copy an RGB buffer to texture.
 * @param rgb_buf the RGB buffer to copy, it must exactly of dimensions width_
 * and height_.
 */
void
SkelGuiTextureDrawer::copy_rgb_to_texture(const unsigned char *rgb_buf)
{
	unsigned char *      row = texture_;
	unsigned char *      tex = texture_;
	const unsigned char *rgb = rgb_buf;
	for (unsigned int h = 0; h < height_; ++h) {
		tex = row;
		for (unsigned int w = 0; w < width_; ++w) {
			*tex++ = *rgb++;
			*tex++ = *rgb++;
			*tex++ = *rgb++;
		}
		row += texture_width_ * 3;
	}
}
