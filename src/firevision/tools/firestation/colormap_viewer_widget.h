
/***************************************************************************
 *  lut_viewer_widget.h - Viwer widget for lookup tables
 *
 *  Created: Thu Mar 20 19:03:02 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
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

class YuvColormap;

class ColormapViewerWidget
{
 public:
  ColormapViewerWidget();
  ~ColormapViewerWidget();

  void set_colormap(YuvColormap* cm);
  void set_colormap_img(Gtk::Image* img);
  
  void draw(unsigned int y_layer = 0);

 private:
  YuvColormap* m_cm;

  Gtk::Image* m_img_colormap;
  unsigned char* m_colormap_img_buf;
};

#endif /* #define __FIREVISION_TOOLS_FIRESTATION_COLORMAP_VIEWER_WIDGET_H_ */
