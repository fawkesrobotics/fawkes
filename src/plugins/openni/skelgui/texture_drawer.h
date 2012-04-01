
/***************************************************************************
 *  texture_drawer.h - Skeleton Visualization GUI: texture drawer
 *
 *  Created: Tue Mar 29 17:06:46 2011 (on the way to Magdeburg for GO2011)
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

#ifndef __PLUGINS_OPENNI_SKELGUI_TEXTURE_DRAWER_H_
#define __PLUGINS_OPENNI_SKELGUI_TEXTURE_DRAWER_H_

namespace firevision {
  class Camera;
}

class SkelGuiTextureDrawer
{
 public:
  SkelGuiTextureDrawer(unsigned int width, unsigned int height);
  virtual ~SkelGuiTextureDrawer();

  virtual void fill_texture() = 0;

  void draw();

 protected:
  void copy_rgb_to_texture(const unsigned char *rgb_buf);

 private:
  unsigned int get_closest_power_of_two(unsigned int n);
  void init_texture();
  void draw_texture();
  void draw_rectangle(float topLeftX, float topLeftY,
		      float bottomRightX, float bottomRightY);

 protected:
  unsigned char       *__texture;	/**< Texture buffer. */

  const unsigned int   __width;		/**< Width of visible area from texture */
  const unsigned int   __height;	/**< Height of visible area from texture */

  const unsigned int   __texture_width;	/**< Real texture width */
  const unsigned int   __texture_height;		/**< Real texture height */

 private:
  bool                 __texture_initialized;
  unsigned int         __texture_id;
  float                __texture_coords[8];

};

#endif
