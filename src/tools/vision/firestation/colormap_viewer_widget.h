
/***************************************************************************
 *  colormap_viewer_widget.h - Viwer widget for colormaps
 *
 *  Created: Thu Mar 20 19:03:02 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __FIREVISION_TOOLS_FIRESTATION_COLORMAP_VIEWER_WIDGET_H_
#define __FIREVISION_TOOLS_FIRESTATION_COLORMAP_VIEWER_WIDGET_H_

#include <gtkmm.h>

namespace firevision {
  class Colormap;
}

class ColormapViewerWidget
{
 public:
  ColormapViewerWidget();
  ~ColormapViewerWidget();

  void set_colormap(firevision::Colormap* cm);
  void set_colormap_img(Gtk::Image* img);
  void set_layer_selector(Gtk::Scale* scl);
  
  void draw(unsigned int layer = 0);

 private:
  bool on_layer_selected(Gtk::ScrollType scroll, double value);

  firevision::Colormap* m_cm;

  Gtk::Image* m_img_colormap;
  Gtk::Scale* m_scl_layer_selector;
  unsigned char* m_colormap_img_buf;
};

#endif /* #define __FIREVISION_TOOLS_FIRESTATION_COLORMAP_VIEWER_WIDGET_H_ */
