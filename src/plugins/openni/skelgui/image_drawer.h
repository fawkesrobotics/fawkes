
/***************************************************************************
 *  image_drawer.h - Skeleton Visualization GUI: image drawer
 *
 *  Created: Sat Mar 19 00:08:08 2011
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

#ifndef __PLUGINS_OPENNI_SKELGUI_IMAGE_DRAWER_H_
#define __PLUGINS_OPENNI_SKELGUI_IMAGE_DRAWER_H_

#include "texture_drawer.h"

namespace firevision {
  class Camera;
}

class SkelGuiImageDrawer : public SkelGuiTextureDrawer
{
 public:
  SkelGuiImageDrawer(firevision::Camera *cam);
  ~SkelGuiImageDrawer();

  void fill_texture();

 private:
  firevision::Camera  *__cam;
  unsigned char       *__rgb_buf;
};

#endif
