
/***************************************************************************
 *  depth_drawer.h - Skeleton Visualization GUI: depth drawer
 *
 *  Created: Tue Mar 29 17:03:59 2011 (on the way to Magdeburg for GO2011)
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

#ifndef _PLUGINS_OPENNI_SKELGUI_DEPTH_DRAWER_H_
#define _PLUGINS_OPENNI_SKELGUI_DEPTH_DRAWER_H_

#include "texture_drawer.h"

namespace firevision {
  class Camera;
}

class SkelGuiDepthDrawer : public SkelGuiTextureDrawer
{
 public:
  SkelGuiDepthDrawer(firevision::Camera *depth_cam, firevision::Camera *label_cam,
		     unsigned int max_depth);
  ~SkelGuiDepthDrawer();

  virtual void fill_texture();

  void toggle_show_labels();

 private:
  firevision::Camera  *depth_cam_;
  firevision::Camera  *label_cam_;
  unsigned char       *rgb_buf_;

  const unsigned int   max_depth_;
  float               *histogram_;

  bool                 show_labels_;
};

#endif
